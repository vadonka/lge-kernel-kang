#!/bin/bash

rm -rf linux-stable-cherry-pick.sh

echo -e "#!/bin/bash\n" > linux-stable-cherry-pick.sh
echo -n "Version to merge: "

read fullversion
version=`echo $fullversion | awk 'BEGIN {FS="."} {print $1}'`
patchlevel=`echo $fullversion | awk 'BEGIN {FS="."} {print $2}'`
sublevel=`echo $fullversion | awk 'BEGIN {FS="."} {print $3}'`
prevsublevel=$(($sublevel-1))
prevversion=$version.$patchlevel.$prevsublevel

echo -e "# Cherry pick commits from kernel $fullversion\n" >> linux-stable-cherry-pick.sh

for commit in `git log v$prevversion...v$fullversion --pretty=oneline --reverse | awk 'BEGIN {FS=" "} {print $1}'`; do
	echo "git cherry-pick -x $commit" >> linux-stable-cherry-pick.sh
done

chmod +x linux-stable-cherry-pick.sh