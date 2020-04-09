from math import *

wheelRadius=0.045
encoderCounts=8096
wheelDistance=0.030



cmdX=1
cmdTheta=1
cmdTime=4

xPos=0;
yPos=0;
theta=0;

trueTheta=0
trueX=0
trueY=0

rightVel=(wheelDistance*cmdTheta)/(2*wheelRadius)+cmdX/wheelRadius
leftVel=-(wheelDistance*cmdTheta)/(2*wheelRadius)+cmdX/wheelRadius

currentTime=0

A=wheelRadius*(rightVel-leftVel)/wheelDistance
B=wheelRadius*(rightVel+leftVel)/2

print(rightVel)
print(leftVel)
print(rightVel*wheelRadius)
print(leftVel*wheelRadius)
print("V_t",A)
print("V_f",B)

print("Theory Theta", A*cmdTime)
print("Theory X", B*sin(A*cmdTime)/A)
print("Theory Y", (B-B*cos(A*cmdTime))/A)

oldTheta=theta

rightPos=0
leftPos=0

loop_HZ=8000

for i in range(1, cmdTime*loop_HZ+1):
    rightPos=rightVel*i/loop_HZ
    leftPos=leftVel*i/loop_HZ


    oldTheta=theta
    theta=wheelRadius/wheelDistance*(rightPos-leftPos)
    d_theta=theta-oldTheta
    v_theta=d_theta*loop_HZ

    v_f=wheelRadius*(rightVel+leftVel)/2
    #sqrt(sin^2(x)+(1-cos(x))^2)=2*abs(sin(x/2))=2*abs(x/2) when x is tiny
    #d_f=v_f/v_theta*2*abs(sin(d_theta/2))
    d_f=v_f/v_theta*2*abs(d_theta/2)
    xPos+=d_f*cos(theta)
    yPos+=d_f*sin(theta)

    '''
    float old_theta=theta;
    theta=config_.wheelRadius/config_.wheelDistance*(encoders[0].pos_estimate_-encoders[1].pos_estimate_);
    if(config_.){
        theta=-theta;
    }
    float d_theta=theta-old_theta;
    float d_f=(config_.wheelRadius*(encoders[0].vel_estimate_+encoders[1].vel_estimate_)/2.0f) //forward velocity
    /
    ((config_.wheelRadius/config_.wheelDistance*(encoders[0].vel_estimate_-encoders[1].vel_estimate_)) //angular velocity
    *
    2.0f*fabsf(d_theta*0.5f)); //sqrt(sin^2(x)+(1-cos(x))^2)=2*abs(sin(x/2))
    x+=d_f*our_arm_cos_f32(theta);
    y+=d_f*our_arm_sin_f32(theta);
    '''

print("Simulated Theta", theta)
print("Simulated X", xPos)
print("Simulated Y", yPos)
print()
print("Differences:")
print("Theta:", abs(theta-A*cmdTime))
print("X:", abs(xPos-B*sin(A*cmdTime)/A))
print("Y:", abs(yPos-(B-B*cos(A*cmdTime))/A))
print("Total Samples:", loop_HZ*cmdTime)
print()
print()

xPos=0;
yPos=0;
theta=0;

wheelAngularDistancePerCount=(2*pi/encoderCounts)
wheelDistancePerCount=wheelAngularDistancePerCount*wheelRadius

time_stepRight=abs(wheelAngularDistancePerCount/rightVel)
time_stepLeft=abs(wheelAngularDistancePerCount/leftVel)

nextStepRightTime=time_stepRight
nextStepLeftTime=time_stepLeft

dThetaPerCount=atan(wheelDistancePerCount/wheelDistance)

totalSamples=0

while(min(nextStepLeftTime, nextStepRightTime)<cmdTime):
    if(nextStepRightTime<nextStepLeftTime):
        nextStepRightTime+=time_stepRight
        if(rightVel>0):
            theta+=dThetaPerCount
        else:
            theta-=dThetaPerCount
    else:
        nextStepLeftTime+=time_stepLeft
        if(leftVel>0):
            theta-=dThetaPerCount
        else:
            theta+=dThetaPerCount
    xPos+=wheelDistancePerCount/2*cos(theta)
    yPos+=wheelDistancePerCount/2*sin(theta)
    totalSamples+=1


print("Simulated Theta", theta)
print("Simulated X", xPos)
print("Simulated Y", yPos)
print()
print("Differences:")
print("Theta:", abs(theta-A*cmdTime))
print("X:", abs(xPos-B*sin(A*cmdTime)/A))
print("Y:", abs(yPos-(B-B*cos(A*cmdTime))/A))
print("Total Samples:", totalSamples)
