#!/usr/bin/expect -f
log_user 1

set timeout 10;

set dir {~/barzin_catkin_ws/src/path_tracking/scripts}
set ip {192.168.4.11}

proc authenticate {} {
    expect {
        "assword:" {
            send -- "[read [open "../Husky_password.txt" r]]\r"

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
spawn bash -c "scp -r administrator@$ip:$dir/experimental_results/*.npy ./src/experimental_results/"
authenticate

send "\n"
spawn bash -c "scp -r ./src/Modules/*.py administrator@$ip:$dir/Modules/"
authenticate

send "\n"
spawn bash -c "scp -r ./src/*.py administrator@$ip:$dir"
authenticate

send "\n"
spawn ssh administrator@$ip
authenticate
expect "administrator"
send "chmod +x $dir/*.py\r"
send "source ~/barzin_catkin_ws/devel/setup.bash\r"
interact