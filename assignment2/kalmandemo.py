import numpy as np

# simulated data
#t = np.linspace(1,100,100) time from 1 to 100
#a = 0.5                    acceleration
#position = (a * t**2)/2    distance

position_noise = position+np.random.normal(0,120,size=(t.shape[0]))  #noise
#import matplotlib.pyplot as plt    
#plt.plot(t,position,label='truth position')                         #plot graph
#plt.plot(t,position_noise,label='only use measured position')


#inital predict postion
predicts = [position_noise[0]]
position_predict = predicts[0]

predict_var = 0
odo_var = 120**2 #The variance of odometry 
v_std = 50 #The variance of laser sensor
for i in range(1,t.shape[0]):
  
    dv =  (position[i]-position[i-1]) + np.random.normal(0,50) # The distance between current position and previous position plus some normal distrbution?
    position_predict = position_predict + dv # predict current postion
    predict_var += v_std**2 # update the variance of predict data 
    #The main kalman filter
    position_predict = position_predict*odo_var/(predict_var + odo_var)+position_noise[i]*predict_var/(predict_var + odo_var) #This formula is somehow calculate the predict position? Found it on the internet.
    predict_var = (predict_var * odo_var)/(predict_var + odo_var)**2  #variance of predict data
    predicts.append(position_predict)    

    
#plt.plot(t,predicts,label='kalman filtered position')

#plt.legend()                            #plot
#plt.show()