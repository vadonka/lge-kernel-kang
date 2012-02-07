#!/bin/bash

fullversion=2.6.32.58
prevversion=2.6.32.57

rm -rf linux-stable-cherry-pick.sh

echo -e "#!/bin/bash\n" > linux-stable-cherry-pick.sh
echo -e "# Cherry pick commits from kernel $fullversion\n" >> linux-stable-cherry-pick.sh

for commit in `git log v$prevversion...v$fullversion --pretty=oneline --reverse | awk 'BEGIN {FS=" "} {print $1}'`; do
	echo "git cherry-pick -x $commit" >> linux-stable-cherry-pick.sh
done

chmod +x linux-stable-cherry-pick.sh