#!/usr/bin/expect -f
log_user 1

set timeout 10;

set dir {~/barzin_catkin_ws/src/path_tracking/scripts}

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
spawn bash -c "scp -r administrator@192.168.4.11:$dir/experimental_results/*.npy ./src/experimental_results/"
authenticate

send "\n"
spawn bash -c "scp -r ./src/*.py administrator@192.168.4.11:$dir"
authenticate

send "\n"
spawn ssh administrator@192.168.4.11
authenticate
expect "administrator"
send "rm $dir/experimental_results/*.npy\r"
send "chmod +x $dir/*.py\r"
send "cd ~/barzin_catkin_ws\r"
send "catkin_make\r"
send "source devel/setup.bash\r"
# send "rosrun path_tracking path_tracker_experiment.py\r"
send "rosrun path_tracking mission.py"

interact