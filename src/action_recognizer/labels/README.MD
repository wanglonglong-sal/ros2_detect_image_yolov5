grep -o 'name: "[^"]*"' ava_action_list_v2.2.pbtxt | sed 's/name: "//;s/"$//' > ava_labels.txt
