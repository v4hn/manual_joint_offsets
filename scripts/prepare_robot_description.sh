#!/bin/sh

xacro_file="$1"
limit="$2"

if [ -z "$xacro_file" ] || [ -z "$limit" ]; then
    echo "Usage: $(basename $0) <xacro_file> <limit>"
    exit 1
fi

xacro "$xacro_file" |
    sed "
    s@\(lower\)=\"[^\\\"]*\"@\1=\"-$limit\"@g;
    s@\(upper\)=\"[^\\\"]*\"@\1=\"$limit\"@g;
    /safety_controller/ d;
    s@type=\"continuous\"@type=\"revolute\"@g;
    T;
    a <limit lower=\"-$limit\" upper=\"$limit\" effort=\"0.0\" velocity=\"0.0\"/>
    "