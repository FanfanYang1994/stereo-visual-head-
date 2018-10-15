function [WGlobal,VGlobal]=coordSaccadeAndVergenceLearn_vrep(W_left,V_left,W_right,V_right,WGlobal,VGlobal,counter,count)  
%
%   [WGlobal,VGlobal]=coordSaccadeAndVergenceLearn_iCub(WLocal,VLocal,WGlobal,VGlobal,size(WGlobal,1),13) ;
tic
%% Yarp network Initialization
disp('Program started');
    % vrep=remApi('remoteApi','extApi.h'); % using the header (requires a compiler)
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID>-1)
        
        disp('Connected');
        
       
        %get handle
       [returnCode,left_eye] = vrep.simxGetObjectHandle(clientID,'lefteye#',vrep.simx_opmode_oneshot_wait);
       [returnCode,right_eye] = vrep.simxGetObjectHandle(clientID,'righteye#',vrep.simx_opmode_oneshot_wait);
       %[returnCode,left_eye]=vrep.simxGetObjectHandle(clientID,'Vision_sensor#',vrep.simx_opmode_oneshot_wait);
       [returnCode,target]=vrep.simxGetObjectHandle(clientID,'Cuboid',vrep.simx_opmode_oneshot_wait);
        disp('get handle ');
        vrep.simxGetVisionSensorImage2(clientID,left_eye,0,vrep.simx_opmode_streaming); % Initialize streaming
        vrep.simxGetVisionSensorImage2(clientID,right_eye,0,vrep.simx_opmode_streaming);
        %get the eye and target position  
        
       [returnCode,target_position]=vrep.simxGetObjectPosition(clientID,target,-1,vrep.simx_opmode_oneshot_wait);%get the target position
       [returnCode,lefteye_position]=vrep.simxGetObjectPosition(clientID,left_eye,-1,vrep.simx_opmode_oneshot_wait);%get the left_eye position
        [returnCode,righteye_position]=vrep.simxGetObjectPosition(clientID,right_eye,-1,vrep.simx_opmode_oneshot_wait);
       % image retrive
        
       [returnCode,resolut,image_left]= vrep.simxGetVisionSensorImage2(clientID,left_eye,0,vrep.simx_opmode_buffer);
       [returnCode,resolut,image_right]= vrep.simxGetVisionSensorImage2(clientID,right_eye,0,vrep.simx_opmode_buffer);% Try to retrieve the streamed data
        %[returnCode]=vrep.simxSetObjectPosition(clientID,target,-1,[0.8,1,0],vrep.simx_opmode_oneshot_wait);
        %code here
        X=target_position(1);
        Y=target_position(2);
        Z=target_position(3);
 
   


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% For cartesian population
%   sigma=[3 3],  centerDiff=5; RF#625
%   sigma=[5 5],  centerDiff=11; RF#121
%   sigma=[7 7],  centerDiff=14; RF#81
%% For logpolar population see gaussianBank_LogPolar()
%%  Initialization
    sigma=[7 7];
    gaussCentDiff=4;  
    stepSize=1;     % 1
    centerDiff=14;
    retinaSize=[128 128];
    SigXtol=sigma(1)+1;
    SigYtol=sigma(2)+1;
    allowedPans=[-20:stepSize:+20]; 
    allowedTilts=[-12:stepSize:+12]; 
    gaussCentersX=[-20:gaussCentDiff:+20]; 
    gaussCentersY=[-12:gaussCentDiff:+12];
    centerPointsX=[SigXtol:centerDiff:retinaSize(1)-SigXtol];
    centerPointsY=[SigYtol:centerDiff:retinaSize(2)-SigYtol];
    [retinaFilters]=gaussian_bank(retinaSize,centerPointsX,centerPointsY,sigma);
    % [retinaFilters]=gaussianBank_LogPolar(retinaSize);
    fovealGaussian=floor(length(retinaFilters)/2)+1;
    % fovealGaussian=1;
    %=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=%
    %% Training Initialization
    epsilon=1e-9;
    mu=0.5;
    sig=0.125;
    N=4*4*length(allowedPans)*length(allowedTilts);
    nC=4
    mC=size(W_left,2)-99; % 4  Number (any arbitrary value) of Global conjunctive neurons 
    if nargin<8, count=1; end
    if nargin<7, counter=0; end
    c=0;
    eps=0;
    PanLPrev=0;TiltLPrev=0;
    PanRPrev=0;TiltRPrev=0;
    % Conjunctive Weigths Initialization
    if nargin<6
        WGlobal=zeros(nC,2*mC); 
        VGlobal=WGlobal;
    end
    disp('Global to Both Eyes iCub Headcentric Training');
    disp(' ');
    %% Headcentric Map Initialization
    headcentPans=[-20:0.1:+20]; 
    headcentTilts=[-12:0.1:+12]; 
    headcentricMap=zeros(length(headcentPans),length(headcentTilts));
    headcentricMap_data=[0 0 0 0 0];
    %% Object world space       
%     Xu=0.14;    %  [0.32:-0.038:-0.326]=   0.3200    0.2820    0.2440    0.2060    0.1680    0.1300    0.0920    0.0540    0.0160   -0.0220   -0.0600   -0.0980
%                 %                          -0.1360   -0.1740   -0.2120   -0.2500   -0.2880   -0.3260         
%     %Xl=-0.08;   
%     Yu=1.041;   %  [1.145:-0.038:0.803]= 1.1450    1.1070    1.0690    1.0310    0.9930    0.9550    0.9170    0.8790    0.8410    0.8030
%     %Yl=0.82;    
%     Z=1.8;  %   1.9000    1.8000    1.7000    1.6000    1.5000    1.4000    1.3000    1.2000    1.1000    1.0000
%             %   0.9000    0.8000    0.7000    0.6000    0.5000    0.4000    0.3000    0.2000
%     X=Xu;
    %=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=%
    pause(15);
    for i=1:2
%         Y=Yu;
%         Zu=Z;
        for j=1:1
%             objs.clear();
%             objs.addDouble(X); 
%             objs.addDouble(Y);  
%             objs.addDouble(Zu); 
%             %send the message
%             objcoorPort.write(objs);
%             %disp('Sent Coordinates=');
%             %disp(objs.toString_c());
%             Zu=Zu-0.2;
%             %Y=Y-0.015;
            [returnCode]=vrep.simxSetObjectPosition(clientID,target,-1,[X,Y,Z],vrep.simx_opmode_oneshot_wait);
            Flag=0;
            Brk=0;
            imFlag=1;
            ydLeft=zeros(mC,1);
            ydRight=zeros(mC,1);
            for k=1:length(allowedPans)
                for l=1:length(allowedTilts)
                    
                    panVal=allowedPans(k)%randi(length(allowedPans)));
                    tiltVal=allowedTilts(l);
                    % iCub gaze shift
                    %[Vg,Vs,Vt]=iCubMotorCommands(panVal,tiltVal,panVal,tiltVal);
                    %headPos.positionMove(5,Vg); % Joint 5 both Eyes vergence
%                     headPos.positionMove(4,panVal); % Joint 4 both Eyes pan/version
%                     headPos.positionMove(3,tiltVal); % Joint 3 both Eyes tilt
                    [returnCode]=vrep.simxSetObjectPosition(clientID,left_eye,-1,lefteye_position+0.01.*[0,panVal,tiltVal],vrep.simx_opmode_oneshot_wait);
                    [returnCode]=vrep.simxSetObjectPosition(clientID,right_eye,-1,righteye_position+0.01.*[0,panVal,tiltVal],vrep.simx_opmode_oneshot_wait);
                    
                    pause(3);   
%                     [retinaLeft]=retinaPreprocess(retinaSize,portLeft);
%                     [retinaRight]=retinaPreprocess(retinaSize,portRight);                        
                    
                    [returnCode,resolut,image]= vrep.simxGetVisionSensorImage2(clientID,left_eye,0,vrep.simx_opmode_buffer); 
                    [imageGray_left,image_left]=createRedMask(image);
                    [retina_Left]=imageGray_left;

                    [returnCode,resolut,image]= vrep.simxGetVisionSensorImage2(clientID,right_eye,0,vrep.simx_opmode_buffer); 
                    [imageGray_right,image_right]=createRedMask(image);
                    [retina_Right]=imageGray_right;



                    if (sum(retina_Left(:)))>0
                        [ydLeft,indL]=eyeActivation(W_left,V_left,retinaFilters,retina_Left,gaussCentersX,gaussCentersY,panVal,tiltVal);
                        %[PanL,TiltL]=eyeMotorCommands(WLocal,VLocal,length(retinaFilters),fovealGaussian,ydLeft,gaussCentersX,gaussCentersY); 
                        ydLeftTemp=ydLeft;
                        %panL=round(PanL);
                        %tiltL=round(TiltL);
                    end
                    if (sum(retina_Right(:)))>0
                        [ydRight,indR]=eyeActivation(W_right,V_right,retinaFilters,retina_Right,gaussCentersX,gaussCentersY,panVal,tiltVal); 
                        %[PanR,TiltR]=eyeMotorCommands(WLocal,VLocal,length(retinaFilters),fovealGaussian,ydRight,gaussCentersX,gaussCentersY); 
                        ydRightTemp=ydRight;
                        %panR=round(PanR);
                        %tiltR=round(TiltR);
                    end
                    %if sum(ydLeft)>0
                    %    if sum(ydRight)>0
                    %xGlobal=[ydLeft;ydRight];
                    %    else
                    %        xGlobal=[ydLeft;ydRight];
                    %    end
                    %else
                    %    if sum(ydRight)>0
                    %        xGlobal=[ydLeft;ydRight];
                    %    end
                    %end
                    xGlobal=[ydLeft;ydRight];
                    if ((sum(ydLeft)>0) && (sum(ydRight))> 0)
                       
                    
                            if sum(xGlobal)>0                                                            
                                   XGlobal=[xGlobal;zeros(size(WGlobal,2)-size(xGlobal,1),1)];  
                                   if counter==0
                                       imFlag=1;
                                   else
                                    [~,rG,~]=conj_act(WGlobal,VGlobal,XGlobal);
                                    ydGlobal=rG(size(xGlobal,1)+1:end);
                                    [~,indG]=max(ydGlobal);
                                    xG=[zeros(size(xGlobal,1),1);ydGlobal];
                                    [~,RG,~]=conj_act(WGlobal,VGlobal,xG);
                                    XG=RG(1:size(xGlobal,1));
                                    ydL=XG(1:length(XG)/2);
                                    ydR=XG((length(XG)/2)+1:end);
                                    [PanL,TiltL]=eyeMotorCommands(W_left,V_left,length(retinaFilters),fovealGaussian,ydL,gaussCentersX,gaussCentersY);
                                    [PanR,TiltR]=eyeMotorCommands(W_right,V_right,length(retinaFilters),fovealGaussian,ydR,gaussCentersX,gaussCentersY);
                                    %% iCub motor commands      
    %                                 [Vg,Vs,Vt]=iCubMotorCommands(PanL,TiltL,PanR,TiltR);
    %                                 headPos.positionMove(5,Vg); % Joint 5 both Eyes vergence
    %                                 headPos.positionMove(4,Vs); % Joint 4 both Eyes pan/version
    %                                 headPos.positionMove(3,Vt); % Joint 3 both Eyes tilt
    %                                 pause(3);   

                                    [returnCode]=vrep.simxSetObjectPosition(clientID,left_eye,-1,lefteye_position+0.01.*[0,PanL,TiltL],vrep.simx_opmode_oneshot_wait);
                                    [returnCode]=vrep.simxSetObjectPosition(clientID,right_eye,-1,righteye_position+0.01.*[0,PanR,TiltR],vrep.simx_opmode_oneshot_wait);


                                    %%
                                 
    %                                 [retinaLeft]=retinaPreprocess(retinaSize,portLeft);
    %                                                     
                                    [returnCode,resolut,image]= vrep.simxGetVisionSensorImage2(clientID,left_eye,0,vrep.simx_opmode_buffer); 
                                    [imageGray_left,image_left]=createRedMask(image);
                                    [retina_Left]=imageGray_left;

                                    [returnCode,resolut,image]= vrep.simxGetVisionSensorImage2(clientID,right_eye,0,vrep.simx_opmode_buffer); 
                                    [imageGray_right,image_right]=createRedMask(image);
                                    [retina_Right]=imageGray_right;

                                    %[retinaRight]=retinaPreprocess(retinaSize,portRight);                    




                                    if (sum(retina_Left(:))>0) && (sum(retina_Right(:))>0)
                                        retRespL=retinal_resp(retinaFilters,retina_Left);
                                        retRespR=retinal_resp(retinaFilters,retina_Right);
                                        if (retRespL(fovealGaussian)>0.7) && (retRespR(fovealGaussian)>0.7)
                                            imFlag=0;                                   
                                        else
                                            imFlag=1;
                                        end
                                    else
                                        imFlag=1;
                                    end
                                %% Headcentric Map formation                            
%                                 if ((sum(ydLeft)>0) && (sum(ydRight))>0)
%                                     panIndL=find(headcentPans==round(PanL));
%                                     tiltIndL=find(headcentTilts==round(TiltL));
%                                     %headcentricMap(tiltIndL,panIndL)=indG;
%                                 else
%                                     if sum(ydLeft)>0
%                                         panIndL=find(headcentPans==round(PanL));
%                                         tiltIndL=find(headcentTilts==round(TiltL));
%                                         %headcentricMap(tiltIndL,panIndL)=indG;
%                                     end
%                                     if sum(ydRight)>0
%                                         panIndR=find(allowedPans==round(PanR+avgHorzDisp));
%                                         if isempty(panIndR) 
%                                             panDiff=round(PanR+avgHorzDisp)-max(allowedPans);
%                                             panIndR=length(allowedPans)+panDiff;
%                                         end
%                                         tiltIndR=find(allowedTilts==round(TiltR));
%                                         %headcentricMap(tiltIndR,panIndR)=indG;
%                                     end
%                                 end
%                                 if find(headcentricMap_data(:,1)~=indG)
%                                     headcentricMap(tiltIndL,panIndL)=indG;
%                                     eps=eps+1;
%                                     headcentricMap_data(eps,:)=[indG PanL TiltL PanR TiltR];
%                                 end
                            end
                        
                           
                            Flag=Flag | imFlag;
                            if imFlag
                                counter=counter+1;
                                %% Conjunction Learning 
                                WGlobal(counter,:)=XGlobal./sum(XGlobal);
                                VGlobal(counter,:)=WGlobal(counter,:)./max(WGlobal(counter,:));
                                %% Disjunctive Learning
                                WGlobal(counter,count+size(xGlobal,1))=1;
                                VGlobal(counter,count+size(xGlobal,1))=1; 
                                               
                            end
%                             if (sum(ydLeft) && sum(ydRight))
%                                 if ((PanL~=PanLPrev)||(TiltL~=TiltLPrev))
%                                     w=w+1;
%                                     horzDisp(w,1)=abs(PanL-PanR);
%                                     Flag=1;
%                                     PanLPrev=PanL;
%                                     TiltLPrev=TiltL;
%                                 end
%                             else
%                                 if sum(ydLeft)
%                                     if ((PanL~=PanLPrev)||(TiltL~=TiltLPrev))
%                                         %Flag=1;
%                                         PanLPrev=PanL;
%                                         TiltLPrev=TiltL;
%                                     end
%                                 end
%                                 if sum(ydRight)
%                                     if ((PanR~=PanRPrev)||(TiltR~=TiltRPrev))
%                                         %Flag=1;
%                                         PanRPrev=PanR;
%                                         TiltRPrev=TiltR;
%                                     end
%                                 end
%                             end
                        end
                    end
                    n=fix(((c)/(N))*100);
                    %waitbar(n);
                   
                    
                    
                end
               
            end
            count=count+1;
%             headPos.positionMove(5,0); % Joint 5 both Eyes vergence
%             headPos.positionMove(4,0); % Joint 4 both Eyes pan/version
%             headPos.positionMove(3,0); % Joint 3 both Eyes tilt
%             pause(3); 
        end
        X=X+0.3;
    end


             vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
        % Now close the connection to V-REP:    
             vrep.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
    vrep.delete(); % call the destructor!
    
    disp('Program ended');
    toc
disp(' ');
%avgHorzDisp=sum(horzDisp)/w
% %% Functions
% function [objcoorPort,objs,portLeft,portRight,headDriver,headPos,headEncs]=yarpInit()
% LoadYarp;
% %=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=%
% %         Yarp Image Grabber and image conversion                         % 
% %=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=%
% import yarp.Port;
% import yarp.Bottle;
% import yarp.BufferedPortImageRgb
% import yarp.BufferedPortBottle;
% portLeft=BufferedPortImageRgb;
% % disp('Registering port /matlab/LeftEye');
% %portLeft.close; % make sure the porDeconvolution_multipath_spare_underwater2a2t is closed, calling open twice hangs 
% portRight=BufferedPortImageRgb;
% % disp('Registering port /matlab/RightEye');
% %portRight.close;
% portLeft.open('/matlab/LeftEye');
% % disp('Please connect the port /matlab/LeftEye to an image source');
% yarp.Network.connect('/icubSim/cam/left/fovea','/matlab/LeftEye');
% portRight.open('/matlab/RightEye');
% % disp('Please connect the port /matlab/RightEye to an image source');
% yarp.Network.connect('/icubSim/cam/right/fovea','/matlab/RightEye');
% %=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=%
% %               iCub Eye Motor Control Initialization                     % 
% %=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=%
% yarp.Network.init();
% %  prepare a property object
% headOptions = yarp.Property();
% headOptions.put('device','remote_controlboard');
% headOptions.put('local','/client/head');
% headOptions.put('remote','/icubSim/head');
% %  create remote driver
% headDriver = yarp.PolyDriver(headOptions);
% headPos = headDriver.viewIPositionControl();
% headEncs = headDriver.viewIEncoders();
% joints=headPos.getAxes();    % get the number of controlled axes in joints
% % %print 'Controlling', 'joints'
% encoders=yarp.Vector(joints);
% headEncs.getEncoders(encoders.data());
% objcoorPort=Port;
% objcoorPort.open('/matlab/objcoordTx');
% %objs=Bottle; 
% objs=Bottle;
% %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [retina]=retinaPreprocess(retinaSize,port)
% yarpImage=0;
% yarpImage=port.read;
% COLOR=zeros(retinaSize);
% [COLOR,MONO]=convert_yarp_image(yarpImage);
% Gray=zeros(retinaSize);
% Gray=rgb2gray(COLOR);
% retina=zeros(retinaSize);
% [retina]=image_preprocess(Gray);
% %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function [Vg,Vs,Vt]=iCubMotorCommands(PanL,TiltL,PanR,TiltR)
% % iCub motor commands 
% Vg=PanL-PanR;
% Vs=(PanL+PanR)/2;
% Vt=(TiltL+TiltR)/2;
