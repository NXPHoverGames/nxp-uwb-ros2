#!/usr/bin/env python3

# Copyright 2023 Google LLC
# Copyright 2025 NXP Semiconductors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
from dataclasses import dataclass, field
import json
from pathlib import Path
import sys
from textwrap import dedent
from typing import List, Tuple, Union, Optional

from pdl import ast, core
from pdl.utils import indent, to_pascal_case

c_prefix = ''

def mask(width: int) -> str:
    return hex((1 << width) - 1)


def deref(var: Optional[str], id: str) -> str:
    return f'{var}.{id}' if var else id


def get_cxx_scalar_type(width: int) -> str:
    """Return the cxx scalar type to be used to back a PDL type."""
    for n in [8, 16, 32, 64]:
        if width <= n:
            return f'uint{n}_t'
    # PDL type does not fit on non-extended scalar types.
    assert False


def get_scalar_size(width: int) -> str:
    """Return the cxx scalar type to be used to back a PDL type."""
    for n in [8, 16, 32, 64]:
        if width <= n:
            return f'{n}'
    # PDL type does not fit on non-extended scalar types.
    assert False


@dataclass
class FieldParser:
    byteorder: str
    offset: int = 0
    shift: int = 0
    extract_arrays: bool = field(default=False)
    chunk: List[Tuple[int, int, ast.Field]] = field(default_factory=lambda: [])
    chunk_nr: int = 0
    unchecked_code: List[str] = field(default_factory=lambda: [])
    code: List[str] = field(default_factory=lambda: [])

    def unchecked_append_(self, line: str):
        """Append unchecked field parsing code.
        The function check_size_ must be called to generate a size guard
        after parsing is completed."""
        self.unchecked_code.append(line)

    def append_(self, line: str):
        """Append field parsing code.
        There must be no unchecked code left before this function is called."""
        assert len(self.unchecked_code) == 0
        self.code.append(line)

    def check_size_(self, size: str):
        """Generate a check of the current span size."""
        self.append_(f"if (size < {size}) {{")
        self.append_("    return false;")
        self.append_("}")

    def check_code_(self):
        """Generate a size check for pending field parsing."""
        if len(self.unchecked_code) > 0:
            #assert len(self.chunk) == 0
            unchecked_code = self.unchecked_code
            self.unchecked_code = []
            self.check_size_(str(self.offset))
            self.code.extend(unchecked_code)
            self.offset = 0

    def parse_bit_field_(self, field: ast.Field):
        """Parse the selected field as a bit field.
        The field is added to the current chunk. When a byte boundary
        is reached all saved fields are extracted together."""

        # Add to current chunk.
        width = core.get_field_size(field)
        self.chunk.append((self.shift, width, field))
        self.shift += width

        # Wait for more fields if not on a byte boundary.
        if (self.shift % 8) != 0:
            return

        # Parse the backing integer using the configured endianness,
        # extract field values.
        size = int(self.shift / 8)
        backing_type = get_cxx_scalar_type(self.shift)

        # Special case when no field is actually used from
        # the chunk.
        should_skip_value = all(isinstance(field, ast.ReservedField) for (_, _, field) in self.chunk)
        if should_skip_value:
            self.unchecked_append_(f"*index += {size}; /* skip reserved fields */")
            self.offset += size
            self.shift = 0
            self.chunk = []
            return

        if len(self.chunk) > 1:
            value = f"chunk{self.chunk_nr}"
            # FIXME swapendianess if self.byteorder is diferent
            self.unchecked_append_(f"{backing_type} {value} = *({backing_type}*)(&payload[*index]);")
            self.chunk_nr += 1
        else:
            if isinstance(field, ast.TypedefField):
                value = f"({field.type_id})(*({backing_type}*)(&payload[*index]))"
            elif size == 3:
                value = f"*({backing_type}*)(&payload[*index]) & 0xffffff"
            else:
                value = f"*({backing_type}*)(&payload[*index])"

        for shift, width, field in self.chunk:
            if isinstance(field, ast.TypedefField):
                shift_type = field.type_id
            else:
                shift_type = backing_type
            v = (value if len(self.chunk) == 1 and shift == 0 else f"({shift_type})((({shift_type}){value} >> {shift}) & {mask(width)})")

            if isinstance(field, ast.ScalarField):
                self.unchecked_append_(f"output->{field.id} = {v};")
            elif isinstance(field, ast.FixedField) and field.enum_id:
                self.unchecked_append_(f"if ({field.enum_id}({v}) != {field.enum_id}::{field.tag_id}) {{")
                self.unchecked_append_("    return false;")
                self.unchecked_append_("}")
            elif isinstance(field, ast.FixedField):
                self.unchecked_append_(f"if (({v}) != {hex(field.value)}) {{")
                self.unchecked_append_("    return false;")
                self.unchecked_append_("}")
            elif isinstance(field, ast.TypedefField):
                self.unchecked_append_(f"output->{field.id} = {v};")
            elif isinstance(field, ast.SizeField):
                self.unchecked_append_(f"{backing_type} {field.field_id}_size = {v};")
            elif isinstance(field, ast.CountField):
                self.unchecked_append_(f"output->{field.field_id}_count = {v};")
            elif isinstance(field, ast.ReservedField):
                pass
            else:
                raise Exception(f'Unsupported bit field type {field.kind}')

        self.unchecked_append_(f"*index += {size};")

        # Reset state.
        self.offset += size
        self.shift = 0
        self.chunk = []

    def parse_typedef_field_(self, field: ast.TypedefField):
        """Parse a typedef field, to the exclusion of Enum fields."""
        if self.shift != 0:
            raise Exception('Typedef field does not start on an octet boundary')

        self.check_code_()
        self.append_(
            dedent("""\
            if (!{c_prefix}parse_{field_type}(payload, (size - *index), index, &output->{field_id})) {{
                return false;
            }}""".format(c_prefix=c_prefix, field_type=field.type.id, field_id=field.id)))

    def parse_optional_field_(self, field: ast.Field):
        """Parse the selected optional field.
        Optional fields must start and end on a byte boundary."""

        self.check_code_()

        if isinstance(field, ast.ScalarField):
            backing_type = get_cxx_scalar_type(field.width)
            self.append_(dedent("""
            if (output->{cond_id} == {cond_value}) {{
                if (size - *index < {size}) {{
                    return false;
                }}

                memcpy(&output->{field_id}, &payload[*index], {size});
                *index += {size};
            }}
            """.format(size=int(field.width / 8),
                       backing_type=backing_type,
                       field_id=field.id,
                       cond_id=field.cond.id,
                       cond_value=field.cond.value,
                       byteorder=self.byteorder)))

        elif isinstance(field, ast.TypedefField) and isinstance(field.type, ast.EnumDeclaration):
            backing_type = get_cxx_scalar_type(field.type.width)
            self.append_(dedent("""
            if ({cond_id} == {cond_value}) {{
                if (size - *index < {size}) {{
                    return false;
                }}
                output->{field_id} = std::make_optional({type_id}(
                    span.read_{byteorder}<{backing_type}, {size}>()));
            }}
            """.format(size=int(field.type.width / 8),
                       backing_type=backing_type,
                       type_id=field.type_id,
                       field_id=field.id,
                       cond_id=field.cond.id,
                       cond_value=field.cond.value,
                       byteorder=self.byteorder)))

        elif isinstance(field, ast.TypedefField):
            self.append_(dedent("""
            if ({cond_id} == {cond_value}) {{
                auto& output = {field_id}_.emplace();
                if (!{type_id}::Parse(span, &output)) {{
                    return false;
                }}
            }}
            """.format(type_id=field.type_id,
                       field_id=field.id,
                       cond_id=field.cond.id,
                       cond_value=field.cond.value)))

        else:
            raise Exception(f"unsupported field type {field.__class__.__name__}")

    def parse_array_field_lite_(self, field: ast.ArrayField):
        """Parse the selected array field.
        This function does not attempt to parse all elements but just to
        identify the span of the array."""
        array_size = core.get_array_field_size(field)
        element_width = core.get_array_element_size(field)
        padded_size = field.padded_size

        if element_width:
            element_width = int(element_width / 8)

        if isinstance(array_size, int):
            size = None
            count = array_size
        elif isinstance(array_size, ast.SizeField):
            size = f'{field.id}_size'
            count = None
        elif isinstance(array_size, ast.CountField):
            size = None
            count = f'output->{field.id}_count'
        else:
            size = None
            count = None

        # Shift the span to reset the offset to 0.
        self.check_code_()

        # Apply the size modifier.
        if field.size_modifier and size:
            self.append_(f"{size} = {size} - {field.size_modifier};")


        # The array size is known in bytes.
        if size is not None:
            self.check_size_(size)

        # The array size is known in bytes.
        if count is not None:
            self.check_size_(count)

        # Compute the array size if the count and element width are known.
        if count is not None and element_width is not None:
            size = f"{count} * {element_width}"

        # Parse from the padded array if padding is present.
        if padded_size:
            self.check_size_(padded_size)
            self.append_("{")
            self.append_(
                f"pdl::packet::slice remaining_span = span.subrange({padded_size}, span.size() - {padded_size});")
            self.append_(f"span = span.subrange(0, {padded_size});")

        if padded_size:
            self.append_(f"span = remaining_span;")
            self.append_("}")

    def parse_array_field_full_(self, field: ast.ArrayField):
        """Parse the selected array field.
        This function does not attempt to parse all elements but just to
        identify the span of the array."""
        array_size = core.get_array_field_size(field)
        element_width = core.get_array_element_size(field)
        element_type = field.type_id or get_cxx_scalar_type(field.width)
        padded_size = field.padded_size

        if element_width:
            element_width = int(element_width / 8)

        if isinstance(array_size, int):
            size = None
            count = array_size
        elif isinstance(array_size, ast.SizeField):
            size = f'{field.id}_size'
            count = None
        elif isinstance(array_size, ast.CountField):
            size = None
            count = f'output->{field.id}_count'
        else:
            size = None
            count = None

        # Shift the span to reset the offset to 0.
        self.check_code_()

        # Apply the size modifier.
        if field.size_modifier and size:
            self.append_(f"{size} = {size} - {field.size_modifier};")

        # Compute the array size if the count and element width are known.
        if count is not None and element_width is not None:
            size = f"{count} * {element_width}"

        # Parse from the padded array if padding is present.
        if padded_size:
            self.check_size_(padded_size)
            self.append_("{")
            self.append_(
                f"pdl::packet::slice remaining_span = span.subrange({padded_size}, span.size() - {padded_size});")
            self.append_(f"span = span.subrange(0, {padded_size});")

        # The array count is known statically, elements are scalar.
        if field.width and field.size:
            assert size is not None
            self.check_size_(size)
            element_size = int(field.width / 8)
            self.append_(f"for (size_t n = 0; n < {field.size}; n++) {{")
            self.append_(f"    output->{field.id}[n] = *({element_type}*)(&payload[*index]);")
            self.append_(f"    *index += {element_size};")
            self.append_("}")

        # The array count is known statically, elements are enum values.
        elif isinstance(field.type, ast.EnumDeclaration) and field.size:
            assert size is not None
            self.check_size_(size)
            element_size = int(field.type.width / 8)
            backing_type = get_cxx_scalar_type(field.type.width)
            self.append_(f"for (size_t n = 0; n < {field.size}; n++) {{")
            self.append_(
                f"    {field.id}_[n] = {element_type}(span.read_{self.byteorder}<{backing_type}, {element_size}>());")
            self.append_("}")

        # The array count is known statically, elements have variable size.
        elif field.size:
            self.append_(f"for (size_t n = 0; n < {field.size}; n++) {{")
            self.append_(f"    if (!{element_type}::Parse(span, &{field.id}_[n])) {{")
            self.append_("        return false;")
            self.append_("    }")
            self.append_("}")

        # The array count is known. The element width is dynamic.
        # Parse each element iteratively and derive the array span.
        elif count is not None and field.type_id is not None:
            self.append_(f"if ({count} > 0) {{")
            self.append_(f"    output->{field.id} = ({field.type_id}*)malloc(sizeof({field.type_id}) * {count});")
            self.append_(f"    if (!output->{field.id}) {{")
            self.append_(f"        return false;")
            self.append_(f"    }}")
            self.append_(f"    for (size_t n = 0; n < {count}; n++) {{")
            self.append_(f"        {element_type} element;")
            self.append_(f"        if (!{c_prefix}parse_{field.type_id}(payload, size, index, &element)) {{")
            self.append_("            return false;")
            self.append_("        }")
            self.append_(f"        output->{field.id}[n] = element;")
            self.append_("     }")
            self.append_(f"}} else {{")
            self.append_(f"    output->{field.id} = 0;")
            self.append_(f"}}")

        # The array size is known in bytes.
        elif size is not None:
            self.check_size_(size)
            if count is not None and element_width is not None:
                self.append_(f"if ({count} > 0) {{")
            else:
                self.append_(f"if ({size} > 0) {{")
            self.append_(f"    output->{field.id} = ({element_type}*)malloc({size});")
            self.append_(f"    if (!output->{field.id}) {{")
            self.append_(f"        return false;")
            self.append_(f"    }}")
            self.append_(f"    memcpy(output->{field.id}, &payload[*index], {size});")
            self.append_(f"    *index += {size};")
            self.append_(f"}} else {{")
            self.append_(f"    output->{field.id} = 0;")
            self.append_(f"}}")

        # The array size is not known, assume the array takes the
        # full remaining space. TODO support having fixed sized fields
        # following the array.
        elif field.width:
            element_size = int(field.width / 8)
            self.append_(f"while (span.size() > 0) {{ test")
            self.append_(f"    if (span.size() < {element_size}) {{")
            self.append_(f"        return false;")
            self.append_("    }")
            self.append_(f"    {field.id}_.push_back(span.read_{self.byteorder}<{element_type}, {element_size}>());")
            self.append_("}")
        elif isinstance(field.type, ast.EnumDeclaration):
            element_size = int(field.type.width / 8)
            backing_type = get_cxx_scalar_type(field.type.width)
            self.append_(f"while (span.size() > 0) {{")
            self.append_(f"    if (span.size() < {element_size}) {{")
            self.append_(f"        return false;")
            self.append_("    }")
            self.append_(
                f"    {field.id}_.push_back({element_type}(span.read_{self.byteorder}<{backing_type}, {element_size}>()));"
            )
            self.append_("}")
        else:
            self.append_(f"while (span.size() > 0) {{")
            self.append_(f"    {element_type} element;")
            self.append_(f"    if (!{element_type}::Parse(span, &element)) {{")
            self.append_(f"        return false;")
            self.append_("    }")
            self.append_(f"    {field.id}_.emplace_back(std::move(element));")
            self.append_("}")

        if padded_size:
            self.append_(f"span = remaining_span;")
            self.append_("}")

    def parse_payload_field_lite_(self, field: Union[ast.BodyField, ast.PayloadField]):
        """Parse body and payload fields."""
        if self.shift != 0:
            raise Exception('Payload field does not start on an octet boundary')

        payload_size = core.get_payload_field_size(field)
        offset_from_end = core.get_field_offset_from_end(field)
        self.check_code_()

        if payload_size and getattr(field, 'size_modifier', None):
            self.append_(f"{field.id}_size -= {field.size_modifier};")

        # The payload or body has a known size.
        # Consume the payload and update the span in case
        # fields are placed after the payload.
        if payload_size:
            self.check_size_(f"{field.id}_size")
            self.append_(f"output->payload = &payload[*index];")
            self.append_(f"output->payload_size = {field.id}_size;")
            self.append_(f"*index += {field.id}_size;")
        # The payload or body is the last field of a packet,
        # consume the remaining span.
        elif offset_from_end == 0:
            self.append_(f"if((size - *index) > 0) {{")
            self.append_(f"    output->payload = &payload[*index];")
            self.append_(f"    output->payload_size = size - *index;")
            self.append_(f"}} else {{")
            self.append_(f"    output->payload_size = 0;")
            self.append_(f"}}")
        # The payload or body is followed by fields of static size.
        # Consume the span that is not reserved for the following fields.
        elif offset_from_end:
            if (offset_from_end % 8) != 0:
                raise Exception('Payload field offset from end of packet is not a multiple of 8')
            offset_from_end = int(offset_from_end / 8)
            self.check_size_(f'{offset_from_end}')
            self.append_(f"payload_ = span.subrange(0, span.size() - {offset_from_end});")
            self.append_(f"span.skip(payload_count);")

    def parse_payload_field_full_(self, field: Union[ast.BodyField, ast.PayloadField]):
        """Parse body and payload fields."""
        if self.shift != 0:
            raise Exception('Payload field does not start on an octet boundary')

        payload_size = core.get_payload_field_size(field)
        offset_from_end = core.get_field_offset_from_end(field)
        self.check_code_()

        if payload_size and getattr(field, 'size_modifier', None):
            self.append_(f"{field.id}_size -= {field.size_modifier};")

        # The payload or body has a known size.
        # Consume the payload and update the span in case
        # fields are placed after the payload.
        if payload_size:
            self.check_size_(f"{field.id}_size")
            self.append_(f"output->payload = (uint8_t*)malloc({field.id}_size);")
            self.append_(f"if(!output->payload) {{")
            self.append_(f"    return false;")
            self.append_(f"}}")
            self.append_(f"memcpy(output->payload, &payload[*index], {field.id}_size);")
            self.append_(f"output->payload_size = {field.id}_size;")
            self.append_(f"*index += {field.id}_size;")
        # The payload or body is the last field of a packet,
        # consume the remaining span.
        elif offset_from_end == 0:
            self.append_("while (span.size() > 0) {")
            self.append_(f"    payload_.push_back(span.read_{self.byteorder}<uint8_t>();")
            self.append_("}")
        # The payload or body is followed by fields of static size.
        # Consume the span that is not reserved for the following fields.
        elif offset_from_end is not None:
            if (offset_from_end % 8) != 0:
                raise Exception('Payload field offset from end of packet is not a multiple of 8')
            offset_from_end = int(offset_from_end / 8)
            self.check_size_(f'{offset_from_end}')
            self.append_(f"while (span.size() > {offset_from_end}) {{")
            self.append_(f"    payload_.push_back(span.read_{self.byteorder}<uint8_t>();")
            self.append_("}")

    def parse(self, field: ast.Field):
        # Field has bit granularity.
        # Append the field to the current chunk,
        # check if a byte boundary was reached.
        if field.cond:
            self.parse_optional_field_(field)

        elif core.is_bit_field(field):
            self.parse_bit_field_(field)

        # Padding fields.
        elif isinstance(field, ast.PaddingField):
            pass

        # Array fields.
        elif isinstance(field, ast.ArrayField) and self.extract_arrays:
            self.parse_array_field_full_(field)

        elif isinstance(field, ast.ArrayField) and not self.extract_arrays:
            self.parse_array_field_lite_(field)

        # Other typedef fields.
        elif isinstance(field, ast.TypedefField):
            self.parse_typedef_field_(field)

        # Payload and body fields.
        elif isinstance(field, (ast.PayloadField, ast.BodyField)) and self.extract_arrays:
            self.parse_payload_field_full_(field)

        elif isinstance(field, (ast.PayloadField, ast.BodyField)) and not self.extract_arrays:
            self.parse_payload_field_lite_(field)

        else:
            raise Exception(f'Unsupported field type {field.kind}')

    def done(self):
        self.check_code_()


@dataclass
class FieldSerializer:
    byteorder: str
    shift: int = 0
    value: List[Tuple[str, int]] = field(default_factory=lambda: [])
    code: List[str] = field(default_factory=lambda: [])
    indent: int = 0

    def indent_(self):
        self.indent += 1

    def unindent_(self):
        self.indent -= 1

    def append_(self, line: str):
        """Append field serializing code."""
        lines = line.split('\n')
        self.code.extend(['    ' * self.indent + line for line in lines])

    def get_payload_field_size(self, var: Optional[str], payload: ast.PayloadField, decl: ast.Declaration) -> str:
        """Compute the size of the selected payload field, with the information
        of the builder for the selected declaration. The payload field can be
        the payload of any of the parent declarations, or the current declaration."""

        if payload.parent.id == decl.id:
            return deref(var, 'packet->payload_size')

        # Get the child packet declaration that will match the current
        # declaration further down.
        child = decl
        while child.parent_id != payload.parent.id:
            child = child.parent

        # The payload is the result of serializing the children fields.
        constant_width = 0
        variable_width = []
        for f in child.fields:
            field_size = core.get_field_size(f)
            if field_size is not None:
                constant_width += field_size
            elif isinstance(f, (ast.PayloadField, ast.BodyField)):
                variable_width.append(self.get_payload_field_size(var, f, decl))
            elif isinstance(f, ast.TypedefField):
                variable_width.append(f"{f.id}_.GetSize()")
            elif isinstance(f, ast.ArrayField):
                #packet->{field.field_id}_count
                variable_width.append(f"packet->{f.id}_count")
            else:
                raise Exception("Unsupported field type")

        constant_width = int(constant_width / 8)
        if constant_width and not variable_width:
            return str(constant_width)

        temp_var = f'{payload.parent.id.lower()}_payload_size'
        self.append_(f"size_t {temp_var} = {constant_width};")
        for dyn in variable_width:
            self.append_(f"{temp_var} += {dyn};")
        return temp_var

    def serialize_array_element_(self, field: ast.ArrayField, var: str):
        """Serialize a single array field element."""
        if field.width:
            backing_type = get_cxx_scalar_type(field.width)
            element_size = int(field.width / 8)
            self.append_(
                f"output[index] = write_{self.byteorder}<{backing_type}, {element_size}>(output, {var});")
        elif isinstance(field.type, ast.EnumDeclaration):
            backing_type = get_cxx_scalar_type(field.type.width)
            element_size = int(field.type.width / 8)
            self.append_(f"*({backing_type}*)&output[index] = {var};")
            self.append_(f"index += {element_size};")
        else:
            self.append_(f"index += {c_prefix}serialize_{field.type.id}(&{var}, &output[index]);")

    def serialize_array_field_(self, field: ast.ArrayField, var: str):
        """Serialize the selected array field."""
        if field.padded_size:
            self.append_(f"size_t {field.id}_end = output.size() + {field.padded_size};")

        if field.size:
            self.append_(f"for (size_t n = 0; n < {field.size}; n++) {{")
            self.indent_()
            self.append_(f"output[index] = {var}[n];")
            self.append_(f"index += {int(field.width / 8)};") #FIXME DOESNT WORK IF ELEMENT IS BIGGER
            self.unindent_()
            self.append_("}")
        elif field.width == 8:
            self.append_(f"for (size_t n = 0; n < {var}_count; n++) {{")
            self.indent_()
            self.append_(f"output[index] = {var}[n];")
            self.append_(f"index += 1;")
            self.unindent_()
            self.append_("}")
        else:
            self.append_(f"for (size_t n = 0; n < {var}_count; n++) {{")
            self.indent_()
            self.serialize_array_element_(field, f'{var}[n]')
            self.unindent_()
            self.append_("}")

        if field.padded_size:
            self.append_(f"while (output.size() < {field.id}_end) {{")
            self.append_("    output.push_back(0);")
            self.append_("}")

    def serialize_bit_field_(self, field: ast.Field, parent_var: Optional[str], var: Optional[str],
                             decl: ast.Declaration):
        """Serialize the selected field as a bit field.
        The field is added to the current chunk. When a byte boundary
        is reached all saved fields are serialized together."""

        # Add to current chunk.
        width = core.get_field_size(field)
        shift = self.shift

        if field.cond_for:
            value_present = field.cond_for.cond.value
            value_absent = 0 if field.cond_for.cond.value else 1
            self.value.append((f"(packet->{field.cond_for.cond.id} == {field.cond_for.cond.value} ? {value_present} : {value_absent})", shift))
        elif isinstance(field, ast.ScalarField):
            self.value.append((f"{var} & {mask(field.width)}", shift))
        elif isinstance(field, ast.FixedField) and field.enum_id:
            self.value.append((f"{field.enum_id.upper()}_{field.tag_id}", shift))
        elif isinstance(field, ast.FixedField):
            self.value.append((f"{field.value}", shift))
        elif isinstance(field, ast.TypedefField):
            self.value.append((f"{var}", shift))

        elif isinstance(field, ast.SizeField):
            max_size = (1 << field.width) - 1
            value_field = core.get_packet_field(field.parent, field.field_id)
            size_modifier = ''

            if getattr(value_field, 'size_modifier', None):
                size_modifier = f' + {value_field.size_modifier}'

            if isinstance(value_field, (ast.PayloadField, ast.BodyField)):
                array_size = self.get_payload_field_size(var, field, decl) + size_modifier

            elif isinstance(value_field, ast.ArrayField):
                accessor_name = to_pascal_case(field.field_id)
                array_size = deref(var, f'packet->{field.field_id}_count') + size_modifier

            self.value.append((f"{array_size}", shift))

        elif isinstance(field, ast.CountField):
            max_count = (1 << field.width) - 1
            self.value.append((f"packet->{field.field_id}_count", shift))

        elif isinstance(field, ast.ReservedField):
            pass
        else:
            raise Exception(f'Unsupported bit field type {field.kind}')

        # Check if a byte boundary is reached.
        self.shift += width
        if (self.shift % 8) == 0:
            self.pack_bit_fields_()

    def pack_bit_fields_(self):
        """Pack serialized bit fields."""

        # Should have an integral number of bytes now.
        assert (self.shift % 8) == 0

        # Generate the backing integer, and serialize it
        # using the configured endiannes,
        size = int(self.shift / 8)
        backing_type = get_cxx_scalar_type(self.shift)
        value = [f"({backing_type})((({backing_type}){v[0]}) << {v[1]})" for v in self.value]

        if len(value) == 0:
            self.append_(f"*({backing_type}*)&output[index] = 0;")
        elif len(value) == 1:
            self.append_(f"*({backing_type}*)&output[index] = ({value[0]});")
        else:
            self.append_(
                f"*({backing_type}*)&output[index] = ({' | '.join(value)});")

        self.append_(f"index += {size};")

        # Reset state.
        self.shift = 0
        self.value = []

    def serialize_typedef_field_(self, field: ast.TypedefField, var: str):
        """Serialize a typedef field, to the exclusion of Enum fields."""

        if self.shift != 0:
            raise Exception('Typedef field does not start on an octet boundary')
        if (isinstance(field.type, ast.StructDeclaration) and field.type.parent_id is not None):
            raise Exception('Derived struct used in typedef field')

        self.append_(f"index += {c_prefix}serialize_{field.type.id}(&{var}, &output[index]);")

    def serialize_optional_field_(self, field: ast.Field):
        """Serialize optional scalar or typedef fields."""

        if isinstance(field, ast.ScalarField):
            backing_type = get_cxx_scalar_type(field.width)
            self.append_(dedent(
                """
                if (packet->{field_cond_id} == {cond}) {{
                    memcpy((uint8_t*)&output[index], &packet->{field_id}, {size});
                    index += {size};
                }}""".format(field_id=field.id,
                            field_cond_id=field.cond.id,
                            cond=field.cond.value,
                            size=int(field.width / 8),
                            backing_type=backing_type,
                            byteorder=self.byteorder)))

        elif isinstance(field, ast.TypedefField) and isinstance(field.type, ast.EnumDeclaration):
            backing_type = get_cxx_scalar_type(field.type.width)
            self.append_(dedent(
                """
                if ({field_id}_.has_value()) {{
                    pdl::packet::Builder::write_{byteorder}<{backing_type}, {size}>(
                        output, static_cast<{backing_type}>({field_id}_.value()));
                }}""".format(field_id=field.id,
                            size=int(field.type.width / 8),
                            backing_type=backing_type,
                            byteorder=self.byteorder)))

        elif isinstance(field, ast.TypedefField):
            self.append_(dedent(
                """
                if ({field_id}_.has_value()) {{
                    {field_id}_->Serialize(output);
                }}""".format(field_id=field.id)))

        else:
            raise Exception(f"unsupported field type {field.__class__.__name__}")

    def serialize_payload_field_(self, field: Union[ast.BodyField, ast.PayloadField], var: str):
        """Serialize body and payload fields."""

        if self.shift != 0:
            raise Exception('Payload field does not start on an octet boundary')

        self.append_(f"memcpy(&output[index], {var}, {var}_size);");
        self.append_(f"index += {var}_size;");

    def serialize(self, field: ast.Field, decl: ast.Declaration, var: Optional[str] = None):
        field_var = deref(var, f'packet->{field.id}') if hasattr(field, 'id') else None

        if field.cond:
            self.serialize_optional_field_(field)

        # Field has bit granularity.
        # Append the field to the current chunk,
        # check if a byte boundary was reached.
        elif core.is_bit_field(field):
            self.serialize_bit_field_(field, var, field_var, decl)

        # Padding fields.
        elif isinstance(field, ast.PaddingField):
            pass

        # Array fields.
        elif isinstance(field, ast.ArrayField):
            self.serialize_array_field_(field, field_var)

        # Other typedef fields.
        elif isinstance(field, ast.TypedefField):
            self.serialize_typedef_field_(field, field_var)

        # Payload and body fields.
        elif isinstance(field, (ast.PayloadField, ast.BodyField)):
            self.serialize_payload_field_(field, deref(var, 'packet->payload'))

        else:
            raise Exception(f'Unimplemented field type {field.kind}')


def generate_enum_declaration(decl: ast.EnumDeclaration) -> str:
    """Generate the implementation of an enum type."""

    enum_name = decl.id
    enum_type = get_cxx_scalar_type(decl.width)
    tag_decls = []
    for t in decl.tags:
        # Exclude default tags: DEFAULT = ..
        if t.value is not None:
            tag_decls.append(f"{enum_name.upper()}_{t.id} = {hex(t.value)},")

    return dedent("""\

        typedef enum  {{
            {tag_decls}
        }} {enum_name};
        """).format(enum_name=enum_name, enum_type=enum_type, tag_decls=indent(tag_decls, 1))


def generate_enum_to_text(decl: ast.EnumDeclaration) -> str:
    """Generate the helper function that will convert an enum tag to string."""

    enum_name = decl.id
    tag_cases = []
    for t in decl.tags:
        # Exclude default tags: DEFAULT = ..
        if t.value is not None:
            tag_cases.append(f"case {enum_name.upper()}_{t.id}: return \"{t.id}\";")

    return dedent("""\

        static const char* {enum_name}Text({enum_name} tag) {{
            switch (tag) {{
                {tag_cases}
                default:
                    return "Unknown {enum_name}";
            }}
        }}
        """).format(enum_name=enum_name, tag_cases=indent(tag_cases, 2))


def generate_packet_view_field_members(decl: ast.Declaration) -> List[str]:
    """Return the declaration of fields that are backed in the view
    class declaration.

    Backed fields include all named fields that do not have a constrained
    value in the selected declaration and its parents.

    :param decl: target declaration"""

    fields = core.get_unconstrained_parent_fields(decl) + decl.fields
    members = []
    for field in fields:
        if field.cond_for:
            # Scalar fields used as condition for optional fields are treated
            # as fixed fields since their value is tied to the value of the
            # optional field.
            pass
        elif isinstance(field, (ast.PayloadField, ast.BodyField)):
            members.append("const uint8_t* payload;")
            members.append(f"size_t payload_size;")
        elif isinstance(field, ast.ArrayField):
            if field.size:
                members.append(f"{get_cxx_scalar_type(field.width)} {field.id}[{field.size}];")
            elif field.type_id is None:
                members.append(f"{get_cxx_scalar_type(field.width)}* {field.id};")
                members.append(f"size_t {field.id}_count;")
            else:
                members.append(f"{field.type_id}* {field.id};")
                members.append(f"size_t {field.id}_count;")
        elif isinstance(field, ast.ScalarField):
            members.append(f"{get_cxx_scalar_type(field.width)} {field.id};")
        elif isinstance(field, ast.TypedefField) and isinstance(field.type, ast.EnumDeclaration):
            members.append(f"{field.type_id} {field.id};")
        elif isinstance(field, ast.TypedefField):
            members.append(f"{field.type_id} {field.id};")

    return members

def generate_packet_field_destructor(decl: ast.Declaration) -> List[str]:
    """Return the destructor of fields that are backed in the view
    class declaration.

    :param decl: target declaration"""

    fields = core.get_unconstrained_parent_fields(decl) + decl.fields
    members = []
    for field in fields:
        if field.cond_for:
            # Scalar fields used as condition for optional fields are treated
            # as fixed fields since their value is tied to the value of the
            # optional field.
            pass
        elif isinstance(field, ast.ArrayField) and not(field.size):
            if field.type_id:
                element_width = field.width or core.get_declaration_size(field.type)
                if element_width is None:
                    members.append(f"if (in->{field.id}) {{")
                    members.append(f"    {c_prefix}free_{field.type_id}(in->{field.id});")
                    members.append(f"    free(in->{field.id});")
                    members.append("}")
                else:
                    members.append(f"if (in->{field.id}) {{")
                    members.append(f"    free(in->{field.id});")
                    members.append("}")
            else:
                members.append(f"if (in->{field.id}) {{")
                members.append(f"    free(in->{field.id});")
                members.append("}")
    return members    


def generate_packet_field_members(decl: ast.Declaration) -> List[str]:
    """Return the declaration of fields that are backed in the view
    class declaration.

    Backed fields include all named fields that do not have a constrained
    value in the selected declaration and its parents.

    :param decl: target declaration"""

    members = []
    for field in decl.fields:
        if isinstance(field, (ast.PayloadField, ast.BodyField)) and not decl.parent:
            members.append("const uint8_t* payload;")
            members.append("size_t payload_size;")
        elif isinstance(field, ast.ArrayField) and field.size:
            if field.type_id is None:
                members.append(f"{get_cxx_scalar_type(field.width)} {field.id}[{field.size}];")
            else:
                members.append(f"{field.type_id} {field.id}[{field.size}];")
        elif isinstance(field, ast.ArrayField):
            element_type = field.type_id or get_cxx_scalar_type(field.width)
            members.append(f"{element_type}* {field.id};")
            members.append(f"size_t {field.id}_count;")    
        elif isinstance(field, ast.ScalarField):
            members.append(f"{get_cxx_scalar_type(field.width)} {field.id};")
        elif isinstance(field, ast.TypedefField) and isinstance(field.type, ast.EnumDeclaration):
            members.append(f"{field.type_id} {field.id};")
        elif isinstance(field, ast.TypedefField):
            members.append(f"{field.type_id} {field.id};")

    return members


def generate_packet_field_serializers(packet: ast.Declaration) -> List[str]:
    """Generate the code to serialize the fields of a packet builder or struct."""
    serializer = FieldSerializer(byteorder=packet.file.byteorder_short)
    constraints = core.get_parent_constraints(packet)
    constraints = dict([(c.id, c) for c in constraints])
    for field in core.get_packet_fields(packet):
        field_id = getattr(field, 'id', None)
        constraint = constraints.get(field_id, None)
        fixed_field = None
        if constraint and constraint.tag_id:
            fixed_field = ast.FixedField(enum_id=field.type_id,
                                         tag_id=constraint.tag_id,
                                         loc=field.loc,
                                         kind='fixed_field')
            fixed_field.parent = field.parent
        elif constraint:
            fixed_field = ast.FixedField(width=field.width, value=constraint.value, loc=field.loc, kind='fixed_field')
            fixed_field.parent = field.parent
        serializer.serialize(fixed_field or field, packet)
    return serializer.code


def generate_scalar_array_field_accessor(field: ast.ArrayField) -> str:
    """Parse the selected scalar array field."""
    element_size = int(field.width / 8)
    backing_type = get_cxx_scalar_type(field.width)
    byteorder = field.parent.file.byteorder_short
    if field.size:
        return dedent("""\
            for (int n = 0; n < {array_size}; n++) {{
                output->{field_id}[n] = *({backing_type}*)&payload[*index];
            }}""").format(field_id=field.id,
                                        backing_type=backing_type,
                                        element_size=element_size,
                                        array_size=field.size,
                                        byteorder=byteorder)
    else:
        return dedent("""\
                output->{field_id}_count = (size - *index);
            if (output->{field_id}_count > 0) {{
                output->{field_id} = ({backing_type}*)malloc( output->{field_id}_count);
                if(!output->{field_id}) {{
                    return false;
                }}
                memcpy(output->{field_id}, &payload[*index],  output->{field_id}_count);
                *index += output->{field_id}_count;
            }} else {{
                output->{field_id} = 0;
            }}""").format(field_id=field.id,
                                        backing_type=backing_type,
                                        element_size=element_size,
                                        byteorder=byteorder)


def generate_enum_array_field_accessor(field: ast.ArrayField) -> str:
    """Parse the selected enum array field."""
    element_size = int(field.type.width / 8)
    backing_type = get_cxx_scalar_type(field.type.width)
    byteorder = field.parent.file.byteorder_short
    if field.size:
        return dedent("""\
            pdl::packet::slice span = {field_id}_;
            std::array<{enum_type}, {array_size}> elements;
            for (int n = 0; n < {array_size}; n++) {{
                elements[n] = {enum_type}(span.read_{byteorder}<{backing_type}, {element_size}>());
            }}
            return elements;""").format(field_id=field.id,
                                        enum_type=field.type.id,
                                        backing_type=backing_type,
                                        element_size=element_size,
                                        array_size=field.size,
                                        byteorder=byteorder)
    else:
        return dedent("""\
            output->{field_id} = ({enum_type}*)malloc(sizeof({enum_type}) * output->{field_id}_count); 
            for (int n = 0; n < output->{field_id}_count; n++) {{
                output->{field_id}[n] = ({enum_type})(*({backing_type}*)(&payload[*index]));
                *index += {element_size};
            }}""").format(field_id=field.id,
                                        enum_type=field.type_id,
                                        backing_type=backing_type,
                                        element_size=element_size,
                                        byteorder=byteorder)


def generate_typedef_array_field_accessor(field: ast.ArrayField) -> str:
    """Parse the selected typedef array field."""
    if field.size:
        return dedent("""\
            pdl::packet::slice span = {field_id}_;
            std::array<{struct_type}, {array_size}> elements;
            for (int n = 0; n < {array_size}; n++) {{
                {struct_type}::Parse(span, &elements[n]);
            }}
            return elements;""").format(field_id=field.id, struct_type=field.type_id, array_size=field.size)
    else:
        return dedent("""\
                    if (output->{field_id}_count > 0) {{
                    output->{field_id} = ({struct_type}*)malloc(sizeof({struct_type}) * output->{field_id}_count);
                    if(!output->{field_id}) {{
                        return false;
                    }}
                    for (size_t i = 0; (i < output->{field_id}_count && size > *index); i++) {{
                        if (!{c_prefix}parse_{struct_type}(payload, size, index, &output->{field_id}[i])) {{
                            break;
                        }};
                    }}
                }} else {{
                    output->{field_id} = 0;
                }}""").format(c_prefix=c_prefix, field_id=field.id, struct_type=field.type_id)


def generate_array_field_accessor(field: ast.ArrayField):
    """Parse the selected array field."""

    if field.width is not None:
        return generate_scalar_array_field_accessor(field)
    elif isinstance(field.type, ast.EnumDeclaration):
        return generate_enum_array_field_accessor(field)
    else:
        return generate_typedef_array_field_accessor(field)


def generate_array_field_size_getters(decl: ast.Declaration) -> str:
    """Generate size getters for array fields. Produces the serialized
    size of the array in bytes."""

    getters = []
    fields = core.get_unconstrained_parent_fields(decl) + decl.fields
    for field in fields:
        if not isinstance(field, ast.ArrayField):
            continue

        element_width = field.width or core.get_declaration_size(field.type)
        size = None

        if element_width and field.size:
            size = int(element_width * field.size / 8)
        elif element_width:
            size = f"{field.id}_count * {int(element_width / 8)}"

        if size:
            getters.append(
                dedent("""\
                size_t Get{accessor_name}Size() const {{
                    return {size};
                }}
                """).format(accessor_name=to_pascal_case(field.id), size=size))
        else:
            getters.append(
                dedent("""\
                size_t Get{accessor_name}Size() const {{
                    size_t array_size = 0;
                    for (size_t n = 0; n < {field_id}_count; n++) {{
                        array_size += {field_id}_[n].GetSize();
                    }}
                    return array_size;
                }}
                """).format(accessor_name=to_pascal_case(field.id), field_id=field.id))

    return '\n'.join(getters)


def generate_packet_size_getter(decl: ast.Declaration) -> (List[str] , bool):
    """Generate a size getter the current packet. Produces the serialized
    size of the packet in bytes."""

    constant_width = 0
    variable_width = []
    for_loop_actions = []
    for f in core.get_packet_fields(decl):
        field_size = core.get_field_size(f)
        if f.cond:
            if isinstance(f, ast.ScalarField):
                variable_width.append(f"(packet->{f.cond.id} == {f.cond.value} ? {f.width} : 0)")
            elif isinstance(f, ast.TypedefField) and isinstance(f.type, ast.EnumDeclaration):
                variable_width.append(f"(packet->{f.cond.id} == {f.cond.value} ? {f.type.width} : 0)")
            elif isinstance(f, ast.TypedefField):
                variable_width.append(f"(packet->{f.cond.id} == {f.cond.value} ? {f.id}_->GetSize() : 0)")
            else:
                raise Exception(f"unsupported field type {f.__class__.__name__}")
        elif field_size is not None:
            constant_width += field_size
        elif isinstance(f, (ast.PayloadField, ast.BodyField)):
            variable_width.append("packet->payload_size")
        elif isinstance(f, ast.TypedefField):
            variable_width.append(f"{c_prefix}size_{f.type_id}(&packet->{f.id});")
        elif isinstance(f, ast.ArrayField):
            decl_size = core.get_declaration_size(f.type)

            if isinstance(f.type, ast.EnumDeclaration):
                byte_per_count = int(f.type.width / 8)
                variable_width.append(f"packet->{f.id}_count * {byte_per_count}")
            elif f.type_id is None:
                byte_per_count = int(f.width / 8)
                variable_width.append(f"packet->{f.id}_count * {byte_per_count}")
            elif decl_size is None:
                for_loop_actions.append(f"size_t {f.id}_size = 0;")
                for_loop_actions.append(f"for (int i = 0; i < packet->{f.id}_count; i++) {{")
                for_loop_actions.append(f"   {f.id}_size += {c_prefix}size_{f.type_id}(&packet->{f.id}[i]);")
                for_loop_actions.append(f"}}")
                variable_width.append(f"{f.id}_size")
            else:
                byte_per_count = int(decl_size / 8)
                variable_width.append(f"packet->{f.id}_count * {byte_per_count}")
        else:
            raise Exception("Unsupported field type")

    for_loop = ""
    for loop in for_loop_actions:
        for_loop += loop + "\n"

    constant_width = int(constant_width / 8)
    if not variable_width:
        return [f"return {constant_width};"], True
    elif len(variable_width) == 1 and constant_width:
        return [f"{for_loop}return {variable_width[0]} + {constant_width};"], False
    elif len(variable_width) == 1:
        return [f"{for_loop}return {variable_width[0]};"], False
    elif len(variable_width) > 1 and constant_width:
        return ([f"{for_loop}return {constant_width} + ("] + " +\n    ".join(variable_width).split("\n") + [");"]), False
    elif len(variable_width) > 1:
        return ([f"{for_loop}return ("] + " +\n    ".join(variable_width).split("\n") + [");"]), False
    else:
        assert False


def generate_packet_view_field_accessors(packet: ast.PacketDeclaration) -> List[str]:
    """Return the declaration of accessors for the named packet fields."""

    accessors = []

    # Add accessors for the backed fields.
    fields = core.get_unconstrained_parent_fields(packet) + packet.fields
    for field in fields:
        if field.cond_for:
            # Scalar fields used as condition for optional fields are treated
            # as fixed fields since their value is tied to the value of the
            # optional field.
            pass
        elif isinstance(field, ast.ArrayField):
            accessors.append(
                dedent("""\
                    {accessor}
                """).format(accessor=indent(generate_array_field_accessor(field), 1)))

    # Add accessors for constrained parent fields.
    # The accessors return a constant value in this case.
    for c in core.get_parent_constraints(packet):
        field = core.get_packet_field(packet, c.id)
        if isinstance(field, ast.ScalarField):
            field_type = get_cxx_scalar_type(field.width)
            accessor_name = to_pascal_case(field.id)
            accessors.append(
                dedent("""\
                {field_type} Get{accessor_name}() const {{
                    return {value};
                }}

                """).format(field_type=field_type, accessor_name=accessor_name, value=c.value))
    return "".join(accessors)


def generate_packet_stringifier(decl: ast.PacketDeclaration) -> str:
    """Generate the packet printer. TODO """

    members = []
    
    for field in decl.fields:
        if field.cond:
            members.append(f"if (in->{field.cond.id} == {field.cond.value}) {{")

        if field.cond_for:
            # Scalar fields used as condition for optional fields are treated
            # as fixed fields since their value is tied to the value of the
            # optional field.
            pass
        elif isinstance(field, (ast.PayloadField, ast.BodyField)) and not decl.parent:
            members.append("for(int i = 0; i < in->payload_size; i++) {")
            members.append("    printf(\" %.2x\", in->payload[i]);")
            members.append("}")
            members.append("printf(\"\\n\");");
        elif isinstance(field, ast.ArrayField) and field.size:
            members.append(f"printf(\"{field.id}:\");")
            for i in range(field.size):
                members.append(f"printf(\" %.2x\", in->{field.id}[{i}]);")
            members.append("printf(\"\\n\");");
        elif isinstance(field, ast.ArrayField):
            if field.type_id is None:
                element_type = field.type_id or get_cxx_scalar_type(field.width)
                members.append(f"for(size_t i = 0; i < in->{field.id}_count; i++) {{")
                members.append(f"    printf(\" %.2x\", in->{field.id}[i]);")
                members.append(f"}}")
                members.append("printf(\"\\n\");");
            else:          
                element_type = field.type_id or get_cxx_scalar_type(field.width)
                if isinstance(field.type, ast.EnumDeclaration):
                    members.append(f"for(size_t i = 0; i < in->{field.id}_count; i++) {{")
                    members.append(f"    printf(\"{field.id}[%li]: %s\\n\", i, {field.type_id}Text(in->{field.id}[i]));")
                    members.append(f"}}")
                    members.append("printf(\"\\n\");");
                else:
                    members.append(f"for(size_t i = 0; i < in->{field.id}_count; i++) {{")
                    members.append(f"    {c_prefix}print_{element_type}(&in->{field.id}[i]);")
                    members.append(f"}}")
                    members.append("printf(\"\\n\");");
        elif isinstance(field, ast.ScalarField):
            if field.width >= 8:
                members.append(f"printf(\"{field.id}: %\" PRIu{get_scalar_size(field.width)} \"\\n\", in->{field.id});")
            else:
                members.append(f"printf(\"{field.id}: %.2x\\n\", in->{field.id});")
        elif isinstance(field, ast.TypedefField) and isinstance(field.type, ast.EnumDeclaration):
            members.append(f"printf(\"{field.id}: %s\\n\", {field.type_id}Text(in->{field.id}));")
        elif isinstance(field, ast.TypedefField):
            members.append(f"{c_prefix}print_{field.type_id}(&in->{field.id});")

        if field.cond:
            members.append(f"}}")


    return '\n'.join(members)


def generate_packet_view_field_parsers(packet: ast.PacketDeclaration) -> str:
    """Generate the packet parser. The validator will extract
    the fields it can in a pre-parsing phase. """

    code = []

    # Generate code to check the validity of the parent,
    # and import parent fields that do not have a fixed value in the
    # current packet.
    if packet.parent:
        code.append(
            dedent("""\
            // Check validity of parent packet.
            if (!parent) {
                return false;
            }
            """))
        parent_fields = core.get_unconstrained_parent_fields(packet)
        if parent_fields:
            code.append("// Copy parent field values.")
            for f in parent_fields:
                code.append(f"output->{f.id} = parent->{f.id};")
            code.append("")
        span = "parent.payload_"
    else:
        span = "parent"

    # Validate parent constraints.
    for c in packet.constraints:
        if c.tag_id:
            enum_type = core.get_packet_field(packet.parent, c.id).type_id.upper()
            code.append(
                dedent("""\
                if (parent->{field_id} != {enum_type}_{tag_id}) {{
                    return false;
                }}
                """).format(field_id=c.id, enum_type=enum_type, tag_id=c.tag_id))
        else:
            code.append(
                dedent("""\
                if (parent.{field_id}_ != {value}) {{
                    return false;
                }}
                """).format(field_id=c.id, value=c.value))

    # Parse fields linearly.
    if packet.fields:
        code.append("// Parse packet field values.")
        parser = FieldParser(extract_arrays=False, byteorder=packet.file.byteorder_short)
        for f in packet.fields:
            parser.parse(f)
        parser.done()
        code.extend(parser.code)

    return '\n'.join(code)


def generate_packet_view_friend_classes(packet: ast.PacketDeclaration) -> str:
    """Generate the list of friend declarations for a packet.
    These are the direct children of the class."""

    return [f"friend class {decl.id}_view;" for (_, decl) in core.get_derived_packets(packet, traverse=False)]


def generate_packet_view(packet: ast.PacketDeclaration) -> str:
    """Generate the implementation of the _view class for a
    packet declaration."""

    parent_class = f"{packet.parent.id}_view" if packet.parent else "pdl::packet::slice"
    field_members = generate_packet_view_field_members(packet)
    field_destructor = generate_packet_field_destructor(packet)
    field_accessors = generate_packet_view_field_accessors(packet)
    field_parsers = generate_packet_view_field_parsers(packet)
    friend_classes = generate_packet_view_friend_classes(packet)
    stringifier = generate_packet_stringifier(packet)
    bytes_initializer = f"parent.bytes_" if packet.parent else "parent"

    ret = dedent("")

    if packet.parent:
        ret += dedent("""\
            typedef struct {packet_name}_view {{
                {parent_class}* parent;
                {field_members}
            }} {packet_name}_view;

            """).format(packet_name=packet.id,
                        parent_class=parent_class,
                        bytes_initializer=bytes_initializer,
                        field_members=indent(field_members, 1))

        ret += dedent("""\
            static bool {c_prefix}parse_{packet_name}_view({parent_class}* parent, const uint8_t* payload, size_t size, size_t* index, {packet_name}_view* output) {{
                {field_parsers}
                {field_accessors}
                return true;
            }}
            """).format(c_prefix=c_prefix,
                        packet_name=packet.id,
                        parent_class=parent_class,
                        field_accessors=indent(field_accessors, 1),
                        field_parsers=indent(field_parsers, 2),
                        stringifier=indent(stringifier, 1))
    else:
        ret += dedent("""\
            typedef struct {packet_name}_view {{
                {field_members}
            }} {packet_name}_view;

            """).format(packet_name=packet.id,
                        field_members=indent(field_members, 1))

        ret += dedent("""\
            static bool {c_prefix}parse_{packet_name}_view(const uint8_t* payload, size_t size, size_t* index, {packet_name}_view* output) {{
                {field_parsers}
                {field_accessors}
                return true;
            }}
            """).format(c_prefix=c_prefix,
                        packet_name=packet.id,
                        field_accessors=indent(field_accessors, 1),
                        field_parsers=indent(field_parsers, 2),
                        stringifier=indent(stringifier, 1))

    ret += dedent("""\

    static void {c_prefix}print_{packet_name}_view({packet_name}_view* in) {{
        {stringifier}
    }}

    """).format(c_prefix=c_prefix,
                packet_name=packet.id,
                stringifier=indent(stringifier, 1))

    if field_destructor != []:
        ret += dedent("""\
        static void {c_prefix}free_{packet_name}_view({packet_name}_view* in) {{
            {field_destructor}
        }}

        """).format(c_prefix=c_prefix,
                    packet_name=packet.id,
                    field_destructor=indent(field_destructor, 1))
    
    return ret


def generate_packet_constructor(struct: ast.StructDeclaration, constructor_name: str) -> str:
    """Generate the implementation of the constructor for a
    struct declaration."""

    constructor_params = []
    constructor_initializers = []
    inherited_fields = core.get_unconstrained_parent_fields(struct)
    payload_initializer = ''
    parent_initializer = []

    for field in inherited_fields:
        if isinstance(field, ast.ArrayField) and field.size:
            element_type = field.type_id or get_cxx_scalar_type(field.width)
            constructor_params.append(f"std::array<{element_type}, {field.size}> {field.id}")
        elif isinstance(field, ast.ArrayField):
            element_type = field.type_id or get_cxx_scalar_type(field.width)
            constructor_params.append(f"std::vector<{element_type}> {field.id}")
        elif isinstance(field, ast.ScalarField):
            backing_type = get_cxx_scalar_type(field.width)
            constructor_params.append(f"{backing_type} {field.id}")
        elif (isinstance(field, ast.TypedefField) and isinstance(field.type, ast.EnumDeclaration)):
            constructor_params.append(f"{field.type_id} {field.id}")
        elif isinstance(field, ast.TypedefField):
            constructor_params.append(f"{field.type_id} {field.id}")

    for field in struct.fields:
        if field.cond_for:
            pass
        elif isinstance(field, (ast.PayloadField, ast.BodyField)):
            constructor_params.append("std::vector<uint8_t> payload")
            if struct.parent:
                payload_initializer = f"payload_ = std::move(payload);"
            else:
                constructor_initializers.append("payload_(std::move(payload))")
        elif isinstance(field, ast.ArrayField) and field.size:
            element_type = field.type_id or get_cxx_scalar_type(field.width)
            constructor_params.append(f"std::array<{element_type}, {field.size}> {field.id}")
            constructor_initializers.append(f"{field.id}_(std::move({field.id}))")
        elif isinstance(field, ast.ArrayField):
            element_type = field.type_id or get_cxx_scalar_type(field.width)
            constructor_params.append(f"std::vector<{element_type}> {field.id}")
            constructor_initializers.append(f"{field.id}_(std::move({field.id}))")
        elif isinstance(field, ast.ScalarField):
            backing_type = get_cxx_scalar_type(field.width)
            field_type = f"std::optional<{backing_type}>" if field.cond else backing_type
            constructor_params.append(f"{field_type} {field.id}")
            constructor_initializers.append(f"{field.id}_({field.id})")
        elif (isinstance(field, ast.TypedefField) and isinstance(field.type, ast.EnumDeclaration)):
            field_type = f"std::optional<{field.type_id}>" if field.cond else field.type_id
            constructor_params.append(f"{field_type} {field.id}")
            constructor_initializers.append(f"{field.id}_({field.id})")
        elif isinstance(field, ast.TypedefField):
            field_type = f"std::optional<{field.type_id}>" if field.cond else field.type_id
            constructor_params.append(f"{field_type} {field.id}")
            constructor_initializers.append(f"{field.id}_(std::move({field.id}))")

    if not constructor_params:
        return ""

    if struct.parent:
        fields = core.get_unconstrained_parent_fields(struct.parent) + struct.parent.fields
        parent_constructor_params = []
        for field in fields:
            constraints = [c for c in struct.constraints if c.id == getattr(field, 'id', None)]
            if field.cond_for:
                pass
            elif isinstance(field, (ast.PayloadField, ast.BodyField)):
                parent_constructor_params.append("std::vector<uint8_t>{}")
            elif isinstance(field, ast.ArrayField):
                parent_constructor_params.append(f"std::move({field.id})")
            elif isinstance(field, ast.ScalarField) and constraints:
                parent_constructor_params.append(f"{constraints[0].value}")
            elif isinstance(field, ast.ScalarField):
                parent_constructor_params.append(f"{field.id}")
            elif (isinstance(field, ast.TypedefField) and isinstance(field.type, ast.EnumDeclaration) and constraints):
                parent_constructor_params.append(f"{field.type_id}::{constraints[0].tag_id}")
            elif (isinstance(field, ast.TypedefField) and isinstance(field.type, ast.EnumDeclaration)):
                parent_constructor_params.append(f"{field.id}")
            elif isinstance(field, ast.TypedefField):
                parent_constructor_params.append(f"std::move({field.id})")
        parent_constructor_params = ', '.join(parent_constructor_params)
        parent_initializer = [f"{struct.parent_id}Builder({parent_constructor_params})"]

    explicit = 'explicit ' if len(constructor_params) == 1 else ''
    constructor_params = ', '.join(constructor_params)
    constructor_initializers = ', '.join(parent_initializer + constructor_initializers)

    return dedent("""\
        {explicit}{constructor_name}({constructor_params})
            : {constructor_initializers} {{
        {payload_initializer}
    }}""").format(explicit=explicit,
                  constructor_name=constructor_name,
                  constructor_params=constructor_params,
                  payload_initializer=payload_initializer,
                  constructor_initializers=constructor_initializers)


def generate_packet_creator(packet: ast.PacketDeclaration) -> str:
    """Generate the implementation of the creator for a
    struct declaration."""

    constructor_name = f"{packet.id}Builder"
    creator_params = []
    constructor_params = []
    fields = core.get_unconstrained_parent_fields(packet) + packet.fields

    for field in fields:
        if field.cond_for:
            # Scalar fields used as condition for optional fields are treated
            # as fixed fields since their value is tied to the value of the
            # optional field.
            pass
        elif isinstance(field, (ast.PayloadField, ast.BodyField)):
            creator_params.append("std::vector<uint8_t> payload")
            constructor_params.append("std::move(payload)")
        elif isinstance(field, ast.ArrayField) and field.size:
            element_type = field.type_id or get_cxx_scalar_type(field.width)
            creator_params.append(f"std::array<{element_type}, {field.size}> {field.id}")
            constructor_params.append(f"std::move({field.id})")
        elif isinstance(field, ast.ArrayField):
            element_type = field.type_id or get_cxx_scalar_type(field.width)
            creator_params.append(f"std::vector<{element_type}> {field.id}")
            constructor_params.append(f"std::move({field.id})")
        elif isinstance(field, ast.ScalarField):
            backing_type = get_cxx_scalar_type(field.width)
            field_type = f"std::optional<{backing_type}>" if field.cond else backing_type
            creator_params.append(f"{field_type} {field.id}")
            constructor_params.append(f"{field.id}")
        elif (isinstance(field, ast.TypedefField) and isinstance(field.type, ast.EnumDeclaration)):
            field_type = f"std::optional<{field.type_id}>" if field.cond else field.type_id
            creator_params.append(f"{field_type} {field.id}")
            constructor_params.append(f"{field.id}")
        elif isinstance(field, ast.TypedefField):
            field_type = f"std::optional<{field.type_id}>" if field.cond else field.type_id
            creator_params.append(f"{field_type} {field.id}")
            constructor_params.append(f"std::move({field.id})")

    creator_params = ', '.join(creator_params)
    constructor_params = ', '.join(constructor_params)

    return dedent("""\
        static std::unique_ptr<{constructor_name}> Create({creator_params}) {{
            return std::make_unique<{constructor_name}>({constructor_params});
        }}""").format(constructor_name=constructor_name,
                      creator_params=creator_params,
                      constructor_params=constructor_params)


def generate_packet_builder(packet: ast.PacketDeclaration) -> str:
    """Generate the implementation of the Builder class for a
    packet declaration."""

    packet_name = f'{packet.id}'
    parent_class = f'{packet.parent_id}Builder' if packet.parent_id else "pdl::packet::Builder"
    builder_constructor = generate_packet_constructor(packet, constructor_name=packet_name)
    builder_creator = generate_packet_creator(packet)
    field_serializers = generate_packet_field_serializers(packet)
    size_getter, const_size = generate_packet_size_getter(packet)
    array_field_size_getters = generate_array_field_size_getters(packet)

    size_arg = ''

    if not(const_size):
        size_arg = f"{packet_name}_view* packet";

    return dedent("""\

        static size_t {c_prefix}serialize_{packet_name}({packet_name}_view* packet, uint8_t* output) {{
            size_t index = 0;
            {field_serializers}
            return index;
        }}
   
        static const size_t {c_prefix}size_{packet_name}({size_arg})  {{
            {size_getter}
        }}

        """).format(c_prefix=c_prefix,
                    packet_name=packet_name,
                    builder_constructor=builder_constructor,
                    builder_creator=builder_creator,
                    field_serializers=indent(field_serializers, 2),
                    size_getter=indent(size_getter, 1),
                    size_arg=size_arg,
                    array_field_size_getters=indent(array_field_size_getters, 1))

def generate_struct_builder(struct: ast.PacketDeclaration) -> str:
    """Generate the implementation of the Builder class for a
    struct declaration."""

    struct_name = f'{struct.id}'
    parent_class = f'{struct.parent_id}Builder' if struct.parent_id else "pdl::struct::Builder"
    field_serializers = generate_packet_field_serializers(struct)

    return dedent("""\

        static size_t {c_prefix}serialize_{struct_name}({struct_name}* packet, uint8_t* output) {{
            size_t index = 0;
            {field_serializers}
            return index;
        }}
        """).format(c_prefix=c_prefix,
                    struct_name=struct_name,
                    field_serializers=indent(field_serializers, 2))


def generate_struct_field_parsers(struct: ast.StructDeclaration) -> str:
    """Generate the struct parser. The validator will extract
    the fields it can in a pre-parsing phase. """

    code = []
    post_processing = []

    parser = FieldParser(extract_arrays=True, byteorder=struct.file.byteorder_short)
    for f in struct.fields:
        parser.parse(f)
    parser.done()
    code.extend(parser.code)

    code.append("return true;")
    return '\n'.join(code)


def generate_struct_field_destructor(decl: ast.Declaration) -> List[str]:
    """Return the destructor of fields that are backed in the view
    class declaration.

    :param decl: target declaration"""

    fields = core.get_unconstrained_parent_fields(decl) + decl.fields
    members = []
    for field in fields:
        if field.cond_for:
            # Scalar fields used as condition for optional fields are treated
            # as fixed fields since their value is tied to the value of the
            # optional field.
            pass
        elif isinstance(field, (ast.PayloadField, ast.BodyField)):
            members.append("if (in->payload) {")
            members.append("    free(in->payload);")
            members.append("}")
        elif isinstance(field, ast.ArrayField):
            if not(field.size):
                members.append(f"if (in->{field.id}) {{")
                members.append(f"    free(in->{field.id});")
                members.append("}")
        elif isinstance(field, ast.TypedefField):
            element_width = core.get_declaration_size(field.type)
            if element_width is None:
                members.append(f"{c_prefix}free_{field.type_id}(&in->{field.id});")
    return members   

def generate_struct_declaration(struct: ast.StructDeclaration) -> str:
    """Generate the implementation of the class for a
    struct declaration."""

    if struct.parent:
        raise Exception("Struct declaration with parents are not supported")

    struct_constructor = generate_packet_constructor(struct, constructor_name=struct.id)
    field_members = generate_packet_field_members(struct)
    field_destructor = generate_struct_field_destructor(struct)
    field_parsers = generate_struct_field_parsers(struct)
    field_serializers = generate_packet_field_serializers(struct)
    size_getter, const_size = generate_packet_size_getter(struct)
    array_field_size_getters = generate_array_field_size_getters(struct)
    stringifier = generate_packet_stringifier(struct)
    
    size_arg = ''

    if not(const_size):
        size_arg = f"{struct.id}* packet";

    ret = dedent("""\

        typedef struct {struct_name} {{
            {field_members}
        }} {struct_name};

        static bool {c_prefix}parse_{struct_name}(const uint8_t* payload, size_t size, size_t* index, {struct_name}* output) {{
            {field_parsers}
        }}
   
        static const size_t {c_prefix}size_{struct_name}({size_arg})  {{
            {size_getter}
        }}
        """).format(c_prefix=c_prefix,
                    struct_name=struct.id,
                    struct_constructor=struct_constructor,
                    field_members=indent(field_members, 1),
                    field_parsers=indent(field_parsers, 2),
                    field_serializers=indent(field_serializers, 2),
                    size_getter=indent(size_getter, 1),
                    size_arg=size_arg,
                    array_field_size_getters=indent(array_field_size_getters, 1))

    if field_destructor != []:
        ret += dedent("""\

        static void {c_prefix}free_{struct_name}({struct_name}* in) {{
            {field_destructor}
        }}

        """).format(c_prefix=c_prefix,
                    struct_name=struct.id,
                    field_destructor=indent(field_destructor, 1))

    ret += dedent("""\

    static void {c_prefix}print_{struct_name}({struct_name}* in) {{
        {stringifier}
    }};

    """).format(c_prefix=c_prefix,
                struct_name=struct.id,
                stringifier=indent(stringifier, 1))

    return ret


def run(input: argparse.FileType, output: argparse.FileType, prefix: Optional[str], include_header: List[str]):
    global c_prefix
    file = ast.File.from_json(json.load(input))
    core.desugar(file)

    include_header = '\n'.join([f'#include <{header}>' for header in include_header])
    c_prefix = f"{prefix}_" if prefix else ""
    c_define_name = input.name.rsplit('.',1)[0].upper()

    # Disable unsupported features in the canonical test suite.
    skipped_decls = [
        'Packet_Custom_Field_ConstantSize',
        'Packet_Custom_Field_VariableSize',
        'Packet_Checksum_Field_FromStart',
        'Packet_Checksum_Field_FromEnd',
        'Struct_Custom_Field_ConstantSize',
        'Struct_Custom_Field_VariableSize',
        'Struct_Checksum_Field_FromStart',
        'Struct_Checksum_Field_FromEnd',
        'Struct_Custom_Field_ConstantSize_',
        'Struct_Custom_Field_VariableSize_',
        'Struct_Checksum_Field_FromStart_',
        'Struct_Checksum_Field_FromEnd_',
        'PartialParent5',
        'PartialChild5_A',
        'PartialChild5_B',
        'PartialParent12',
        'PartialChild12_A',
        'PartialChild12_B',
    ]

    output.write(
        dedent("""\
        // File generated from {input_name}, with the command:
        //  {input_command}
        // /!\\ Do not edit by hand
        #ifndef __{c_define_name}_H
        #define __{c_define_name}_H
        #include <stdint.h>
        #include <stddef.h>
        #include <stdio.h>
        #include <stdlib.h>
        #include <string.h>
        #include <stdbool.h>
        #include <inttypes.h>
        {include_header}

        #ifdef __cplusplus
        extern "C" {{
        #endif
        """).format(input_name=input.name,
                    c_define_name=c_define_name,
                    input_command=' '.join(sys.argv),
                    include_header=include_header))

    for d in file.declarations:
        if d.id in skipped_decls:
            continue

        if isinstance(d, ast.EnumDeclaration):
            output.write(generate_enum_declaration(d))
            output.write(generate_enum_to_text(d))
        elif isinstance(d, ast.PacketDeclaration):
            output.write(generate_packet_view(d))
            output.write(generate_packet_builder(d))
        elif isinstance(d, ast.StructDeclaration):
            output.write(generate_struct_declaration(d))
            output.write(generate_struct_builder(d))

    output.write(f"#ifdef __cplusplus\n")
    output.write(f"}}\n")
    output.write(f"#endif\n")
    output.write(f"#endif /* __{c_define_name}_H */\n")


def main() -> int:
    """Generate cxx PDL backend."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=argparse.FileType('r'), default=sys.stdin, help='Input PDL-JSON source')
    parser.add_argument('--output', type=argparse.FileType('w'), default=sys.stdout, help='Output C++ file')
    parser.add_argument('--prefix', type=str, help='function prefix')
    parser.add_argument('--include-header', type=str, default=[], action='append', help='Added include directives')
    return run(**vars(parser.parse_args()))


if __name__ == '__main__':
    sys.exit(main())
