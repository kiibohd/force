#!/bin/bash
# Updates all raw.gz files on plotly
# Lists all files in the current working directory
# Jacob Alexander 2016
set -x

DRYRUN=${DRYRUN:-false}
CURDIR=$(cd $(dirname "${BASH_SOURCE[0]}" ) && pwd)

for path in $(ls *.raw.gz); do
	CMD="${CURDIR}/fcv.py --upload --curves 1,2,3,4 ${path} -- plotly"
	echo ${CMD}
	${DRYRUN} || ${CMD}
done

