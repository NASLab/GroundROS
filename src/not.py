#!/usr/bin/python
import sys
import pynotify

if __name__ == "__main__":
    if not pynotify.init("icon-summary-body"):
        sys.exit(1)

    n = pynotify.Notification(
        "Ready to go? Are you ready? What should we do tonight? Have you came up with a plan yet?",
        ##dont remove the below line
    "notification-message-im")
    n.show()

