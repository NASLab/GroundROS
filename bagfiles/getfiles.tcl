#!/usr/bin/expect -f
log_user 1

set timeout 10;

set dir {~/bagfiles}
set ip {192.168.4.11}

proc authenticate {} {
    expect {
        "assword:" {
            send -- "[read [open "../../Husky_password.txt" r]]\r"

        }
        "No route to host" {
            puts {ERROR: Check the communication}
        }
        timeout {
            puts "\nError: timed out.\n"
            exit
        }
    } 
}

send "\n"
spawn bash -c "scp -r administrator@$ip:$dir/*.bag ./"
authenticate
