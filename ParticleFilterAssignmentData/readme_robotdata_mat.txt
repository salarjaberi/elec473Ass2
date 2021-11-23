The following data are stored in the robotdata.mat file

Log.rp(i,:) -> [x_robot y_robot \theta_robot] in ODOMETRY frame at current time step
Log.lp(i,:) -> [x_laser y_laser \theta_laser] in ODOMETRY frame at current time step, NaN if not 'L' data
Log.r(i,:)  -> [ 180 columns of laser data  ], NaN if not 'L' data
Log.ts(i,:) -> timestamp at current time step (can be ignored)

for i = 1:length(Log.rp(:,1))