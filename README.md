# CooperativeLocalization
This package works for a pair of robots who moves as caterpillar-like pattern: one robot moves and the other stays motionless, then the role reverts till both reach the target.
The package fuses two sources of localization data: relative pose detection from the "follower" and the V-SLAM performed by the "leader". The relative pose detection is based on known features(fiducial markers attached on the back side of the leader) while the other localization is based on unknown environmental features.


