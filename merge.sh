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

cc=1
for commit in `git log v$prevversion...v$fullversion --pretty=oneline --reverse | awk 'BEGIN {FS=" "} {print $1}'`; do
	echo "c$cc=\`git log --format=full | grep -c $commit\`" >> linux-stable-cherry-pick.sh
	echo "if [ \"\$c$cc\" = \"0\" ]; then" >> linux-stable-cherry-pick.sh
	echo -e "\tgit cherry-pick -x $commit" >> linux-stable-cherry-pick.sh
	echo "fi" >> linux-stable-cherry-pick.sh
	cc=$(($cc+1))
done

chmod +x linux-stable-cherry-pick.sh
