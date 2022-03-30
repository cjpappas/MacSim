1) GitPod can't run the faster tensor flow because it requires `npm i @tensorflow/tfjs-node` and this version of node (0.19) can't support it.  We need a node updgrade somewhere, probably in the base image.

2) The HUD communicates to the simulation via websockets.  Even when running "in" GitPod, that HUD is still running on a browser on the local machine.  When running in gitpod, we don't have a remapping for `ws://` URIs yet so the connection can't be made.  Thus we need to open in local VS code to get all the port tunneling to work.  If/When we get that to work, we then also need to adjust the `roslaunch` for rosbridge to indicate it is being served on port 80, not the port it is listening on, otherwise you get errors.

3) Why is it firing up two browser windows when we start in gitpod?