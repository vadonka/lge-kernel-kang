#!/bin/bash

# Cherry pick commits from kernel 3.0.31

c1=`git log --format=full | grep -c cb2fee3223eaaeaab386d635fa905ad3925ca539`
if [ "$c1" = "0" ]; then
	git cherry-pick -x cb2fee3223eaaeaab386d635fa905ad3925ca539
fi
c2=`git log --format=full | grep -c 03a9f194904985d2844a2906ba02306289b0b4e7`
if [ "$c2" = "0" ]; then
	git cherry-pick -x 03a9f194904985d2844a2906ba02306289b0b4e7
fi
c3=`git log --format=full | grep -c 95cb2c603f27af05871e3e6718b6e1e1a6f59417`
if [ "$c3" = "0" ]; then
	git cherry-pick -x 95cb2c603f27af05871e3e6718b6e1e1a6f59417
fi
c4=`git log --format=full | grep -c 322fd620a858fab9c1ea85a7cfebe3fc041d7126`
if [ "$c4" = "0" ]; then
	git cherry-pick -x 322fd620a858fab9c1ea85a7cfebe3fc041d7126
fi
c5=`git log --format=full | grep -c d25895e8f1e5e29823d373096d8f3d271bf11821`
if [ "$c5" = "0" ]; then
	git cherry-pick -x d25895e8f1e5e29823d373096d8f3d271bf11821
fi
c6=`git log --format=full | grep -c aa4a6ac6d1eec0fb5c88cbc352a056ae826bc985`
if [ "$c6" = "0" ]; then
	git cherry-pick -x aa4a6ac6d1eec0fb5c88cbc352a056ae826bc985
fi
c7=`git log --format=full | grep -c 5b09471039a7e08329eb1f48f2153bd979188351`
if [ "$c7" = "0" ]; then
	git cherry-pick -x 5b09471039a7e08329eb1f48f2153bd979188351
fi
c8=`git log --format=full | grep -c 7a47462902d03c6e4d3412a5b703069f4dd13c44`
if [ "$c8" = "0" ]; then
	git cherry-pick -x 7a47462902d03c6e4d3412a5b703069f4dd13c44
fi
c9=`git log --format=full | grep -c 17a766decb63b6fe4c43c06069b92b4958d7a642`
if [ "$c9" = "0" ]; then
	git cherry-pick -x 17a766decb63b6fe4c43c06069b92b4958d7a642
fi
c10=`git log --format=full | grep -c 7d841e23feb06283577bd593ecd18113829a837c`
if [ "$c10" = "0" ]; then
	git cherry-pick -x 7d841e23feb06283577bd593ecd18113829a837c
fi
c11=`git log --format=full | grep -c 893127e466a081703d2e2aac572bdad9c22c00ff`
if [ "$c11" = "0" ]; then
	git cherry-pick -x 893127e466a081703d2e2aac572bdad9c22c00ff
fi
c12=`git log --format=full | grep -c 8c9def922a843512c403d348dd033aa301e3eefe`
if [ "$c12" = "0" ]; then
	git cherry-pick -x 8c9def922a843512c403d348dd033aa301e3eefe
fi
c13=`git log --format=full | grep -c fb247af4ccc4b082dbba85d90a42d31fd48affb2`
if [ "$c13" = "0" ]; then
	git cherry-pick -x fb247af4ccc4b082dbba85d90a42d31fd48affb2
fi
c14=`git log --format=full | grep -c e469853fcb813790b1d1152122d83a0f2513fc72`
if [ "$c14" = "0" ]; then
	git cherry-pick -x e469853fcb813790b1d1152122d83a0f2513fc72
fi
c15=`git log --format=full | grep -c 20eae41274bb811063f95a2dde0b3dda88a3d5a0`
if [ "$c15" = "0" ]; then
	git cherry-pick -x 20eae41274bb811063f95a2dde0b3dda88a3d5a0
fi
c16=`git log --format=full | grep -c cb30a2fb48b70262f9989a0d0bbc2a42ec156a22`
if [ "$c16" = "0" ]; then
	git cherry-pick -x cb30a2fb48b70262f9989a0d0bbc2a42ec156a22
fi
c17=`git log --format=full | grep -c 326f0492f8b0d4e3f1e1a7f47eddd0c0f7a644d9`
if [ "$c17" = "0" ]; then
	git cherry-pick -x 326f0492f8b0d4e3f1e1a7f47eddd0c0f7a644d9
fi
c18=`git log --format=full | grep -c 9239fabf848397ec26356b5f267c787840ba4bb7`
if [ "$c18" = "0" ]; then
	git cherry-pick -x 9239fabf848397ec26356b5f267c787840ba4bb7
fi
c19=`git log --format=full | grep -c a674bcab9066a2b2541d8276f5e9ff86f50ce13e`
if [ "$c19" = "0" ]; then
	git cherry-pick -x a674bcab9066a2b2541d8276f5e9ff86f50ce13e
fi
c20=`git log --format=full | grep -c dbe7f938e41ed62242b4dfc1fc77f918646fad5c`
if [ "$c20" = "0" ]; then
	git cherry-pick -x dbe7f938e41ed62242b4dfc1fc77f918646fad5c
fi
c21=`git log --format=full | grep -c d2fd339e9fa7343f521e72add938fd1120f3f8d9`
if [ "$c21" = "0" ]; then
	git cherry-pick -x d2fd339e9fa7343f521e72add938fd1120f3f8d9
fi
c22=`git log --format=full | grep -c 034199be7b5b9efd9b99f34fe2f229f15e166865`
if [ "$c22" = "0" ]; then
	git cherry-pick -x 034199be7b5b9efd9b99f34fe2f229f15e166865
fi
c23=`git log --format=full | grep -c ca288ca1de5957cb50e170dcdb5375c0e6467405`
if [ "$c23" = "0" ]; then
	git cherry-pick -x ca288ca1de5957cb50e170dcdb5375c0e6467405
fi
c24=`git log --format=full | grep -c 197d1155b07b582d9969f456f61ee07d632af7e1`
if [ "$c24" = "0" ]; then
	git cherry-pick -x 197d1155b07b582d9969f456f61ee07d632af7e1
fi
c25=`git log --format=full | grep -c a8eaeff79eb97662e2d06cc1919d902fc251e9da`
if [ "$c25" = "0" ]; then
	git cherry-pick -x a8eaeff79eb97662e2d06cc1919d902fc251e9da
fi
c26=`git log --format=full | grep -c 06200304e7eb237015f433bd8884975e93aba1f5`
if [ "$c26" = "0" ]; then
	git cherry-pick -x 06200304e7eb237015f433bd8884975e93aba1f5
fi
c27=`git log --format=full | grep -c 1cb1976ecd018b02825e5a0fba06ffe95bdaedc6`
if [ "$c27" = "0" ]; then
	git cherry-pick -x 1cb1976ecd018b02825e5a0fba06ffe95bdaedc6
fi
c28=`git log --format=full | grep -c beed6c2e00e0dde6722b590e6a02c20248224c68`
if [ "$c28" = "0" ]; then
	git cherry-pick -x beed6c2e00e0dde6722b590e6a02c20248224c68
fi
c29=`git log --format=full | grep -c 70403b35a5e2d08c9e2727b2e8dd78cb0b1391b3`
if [ "$c29" = "0" ]; then
	git cherry-pick -x 70403b35a5e2d08c9e2727b2e8dd78cb0b1391b3
fi
c30=`git log --format=full | grep -c 62a17c9c34a40907e250b5ac110a5c64325f0aef`
if [ "$c30" = "0" ]; then
	git cherry-pick -x 62a17c9c34a40907e250b5ac110a5c64325f0aef
fi
c31=`git log --format=full | grep -c 0c5f01a4e19099d740d85ea63e2605aa705cdd9e`
if [ "$c31" = "0" ]; then
	git cherry-pick -x 0c5f01a4e19099d740d85ea63e2605aa705cdd9e
fi
c32=`git log --format=full | grep -c cfea9a253443f5b02ea1e0f9667e6cf987069f9b`
if [ "$c32" = "0" ]; then
	git cherry-pick -x cfea9a253443f5b02ea1e0f9667e6cf987069f9b
fi
c33=`git log --format=full | grep -c 34dea1cae3e37fe34ddf7b0f7b581aebcb70db97`
if [ "$c33" = "0" ]; then
	git cherry-pick -x 34dea1cae3e37fe34ddf7b0f7b581aebcb70db97
fi
c34=`git log --format=full | grep -c 9f0c771dfaece0bca44cead53ea6df64e099ae10`
if [ "$c34" = "0" ]; then
	git cherry-pick -x 9f0c771dfaece0bca44cead53ea6df64e099ae10
fi
c35=`git log --format=full | grep -c ca14f0481bc8653c39e7b2fca81bc5131ac9afa8`
if [ "$c35" = "0" ]; then
	git cherry-pick -x ca14f0481bc8653c39e7b2fca81bc5131ac9afa8
fi
c36=`git log --format=full | grep -c 173c412ef8f37d781eb90f5cc8eeab118b987e68`
if [ "$c36" = "0" ]; then
	git cherry-pick -x 173c412ef8f37d781eb90f5cc8eeab118b987e68
fi
c37=`git log --format=full | grep -c ecc53109a021559a49beb81c9c4c859012ee975e`
if [ "$c37" = "0" ]; then
	git cherry-pick -x ecc53109a021559a49beb81c9c4c859012ee975e
fi
c38=`git log --format=full | grep -c c31943d4c5e3facc3a67768090a2f3bf2eb757f3`
if [ "$c38" = "0" ]; then
	git cherry-pick -x c31943d4c5e3facc3a67768090a2f3bf2eb757f3
fi
c39=`git log --format=full | grep -c ad567d13573186f184fb4a2ec0d3a1ca94baedbb`
if [ "$c39" = "0" ]; then
	git cherry-pick -x ad567d13573186f184fb4a2ec0d3a1ca94baedbb
fi
c40=`git log --format=full | grep -c 9051b1e1aed62944eb634c3a85fad76353e5fa05`
if [ "$c40" = "0" ]; then
	git cherry-pick -x 9051b1e1aed62944eb634c3a85fad76353e5fa05
fi
c41=`git log --format=full | grep -c b476e58a834f099aa0f7b4d0a71853e6c61ee6d8`
if [ "$c41" = "0" ]; then
	git cherry-pick -x b476e58a834f099aa0f7b4d0a71853e6c61ee6d8
fi
c42=`git log --format=full | grep -c b00c5b8d8590a0fe2713a4bee24e9562480ccb9e`
if [ "$c42" = "0" ]; then
	git cherry-pick -x b00c5b8d8590a0fe2713a4bee24e9562480ccb9e
fi
c43=`git log --format=full | grep -c 9bd46fe16654ee5a10dc269ebe3fc44903424707`
if [ "$c43" = "0" ]; then
	git cherry-pick -x 9bd46fe16654ee5a10dc269ebe3fc44903424707
fi
c44=`git log --format=full | grep -c e4aef4293263c455992c4943fe53b8045209f383`
if [ "$c44" = "0" ]; then
	git cherry-pick -x e4aef4293263c455992c4943fe53b8045209f383
fi
c45=`git log --format=full | grep -c 137c55d525bbdc509cb59abbc48fc04ba7b6a0da`
if [ "$c45" = "0" ]; then
	git cherry-pick -x 137c55d525bbdc509cb59abbc48fc04ba7b6a0da
fi
c46=`git log --format=full | grep -c aef49be82379e995b42648f36dd02d70f979ef2a`
if [ "$c46" = "0" ]; then
	git cherry-pick -x aef49be82379e995b42648f36dd02d70f979ef2a
fi
c47=`git log --format=full | grep -c 7bfac470b517b18d496e96acc90be58353df2159`
if [ "$c47" = "0" ]; then
	git cherry-pick -x 7bfac470b517b18d496e96acc90be58353df2159
fi
c48=`git log --format=full | grep -c 8792953929b01e82e57bce07cc717a0bf24384bc`
if [ "$c48" = "0" ]; then
	git cherry-pick -x 8792953929b01e82e57bce07cc717a0bf24384bc
fi
c49=`git log --format=full | grep -c bea37381fd9a34c6660e5195d31beea86aa3dda3`
if [ "$c49" = "0" ]; then
	git cherry-pick -x bea37381fd9a34c6660e5195d31beea86aa3dda3
fi
