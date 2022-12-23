#!usr/bin/bash
while read -r line; do 'atom' $line; sleep 0.3; done < paths
