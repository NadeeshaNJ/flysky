#!/usr/bin/env python3
"""Reset the Kinect over USB to recover a stuck/half-claimed device.

The libfreenect sync API leaves the Kinect's USB interfaces claimed if a process
exits without calling ``freenect.sync_stop()`` (e.g. a hard kill). The camera /
audio subdevices can then fail to open, or drop off the bus entirely. This issues
a real USBDEVFS_RESET to every Microsoft (045e) node, which re-enumerates the
Kinect's internal hub and brings the camera + audio back.

Usage:
    sudo python3 scripts/kinect_usb_reset.py
"""

import fcntl
import glob
import os

USBDEVFS_RESET = 0x5514  # _IO('U', 20)


def reset(path: str) -> None:
    fd = os.open(path, os.O_WRONLY)
    try:
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
        print(f"reset OK: {path}")
    finally:
        os.close(fd)


def main() -> None:
    targets = []
    for dev in glob.glob('/sys/bus/usb/devices/*'):
        try:
            with open(os.path.join(dev, 'idVendor')) as f:
                if f.read().strip() != '045e':
                    continue
            busnum = int(open(os.path.join(dev, 'busnum')).read())
            devnum = int(open(os.path.join(dev, 'devnum')).read())
        except OSError:
            continue
        targets.append('/dev/bus/usb/%03d/%03d' % (busnum, devnum))

    if not targets:
        print('No Kinect (045e) devices found on the USB bus.')
        return

    for path in sorted(targets):
        try:
            reset(path)
        except PermissionError:
            print(f'reset FAILED {path}: permission denied (run with sudo)')
        except OSError as exc:
            print(f'reset FAILED {path}: {exc}')


if __name__ == '__main__':
    main()
