#! /bin/bash
dts devel run -H bestduckbot.local -- --privileged
dts devel build -f -H bestduckbot.local

echo "buildin ja runnin, sir!"
