Sptam_coop is a only vision-based localization method, it is a modified version of SPTAM. 

For more infos about SPTAM you could just go to the SPTAM package which has been also uploaded. 

The difference lies in the introduced fusion mechanism. Our robot group has two robots: Apollo and Boreas. Boreas is equiped with stereo camera and Apollo HD camera. The backside of Boreas is mounted with a 2*2 aruco marker which is for relative pose detection. Boreas performs normal V-SLAM while at the same time receives the relative pose detection results from Apollo.

This packages fuses this two sources of localization data and produces a more robust and accurate localization results: When the environment has a bunch of features, our algorithm helps build a more accurate map which helps further localize the robots better; When the environment has deficient features which is desastrous for feature-based localization algorithm, our method could still localize the robots robustly thanks to the relative pose detection. When the environment has ambiguous features our algorithm could differenciate similar features due to the extra constrains introduced by the relative pose detection and thus creates more robust localization results.
