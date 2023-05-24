function t = convertDateTimeToSeconds(DTquery,DT,time)
t = interp1(DT,time,DTquery);