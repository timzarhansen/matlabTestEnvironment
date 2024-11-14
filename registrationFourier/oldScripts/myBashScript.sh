 #!/bin/bash  

args=("$@")  
 echo Number of arguments passed: $#
 for var in "$@"
 do
    echo "$var"
 done

source ~/.bashrc

rosrun underwaterslam registrationStPereDataset 1
