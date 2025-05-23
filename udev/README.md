udev rules for Linux
====================

On Linux, permission to access device drivers from user space must be explicitly granted
via udev rules. This also is valid for the srxxx spi driver.

First we need to add <user> to the GROUP 'plugdev' 

```
$ sudo usermod -a -G plugdev <user>
```
Now continue with adding a new rules file to udev
To install, copy the rules file `srxxx.rules` in this directory to `/etc/udev/rules.d/`

```
$ sudo cp srxxx.rules /etc/udev/rules.d
```

To see your changes without a reboot, you can force the udev system to reload:

```
$ sudo udevadm control --reload
$ sudo udevadm trigger
```
Now the srxxx spi driver get's the right permissions during startup.
