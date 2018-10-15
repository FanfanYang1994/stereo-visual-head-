function [reponseLeft,reponseRight,headcentricMap,y,inputXL,rb,y1,PanL,TiltL]=test_saccadeControl_vrep(Wleft,Vleft,Wright,Vright,WGlobal,VGlobal)  
%% Yarp network Initialization
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
%% Headcentric Map
headcentricMap=zeros(length(allowedTilts),length(allowedPans));
avgHorzDisp=7.03;
%% Test Initialization
disp('Global to Both Eyes iCub Saccade Testing.......');
disp(' ');
%% Receptive Fields
nd=60;
% for i=1:nd
%   disj_nodeRespMax{i}=zeros(8,6);
%   disj_nodeRespMin{i}=10.*ones(8,6);
% end
%% Object world space
% Xrand=[0.1400:-0.015:-0.07];    %  0.1400    0.1250    0.1100    0.0950    0.0800    0.0650    0.0500    0.0350    
%                                 %  0.0200    0.0050    -0.0100   -0.0250   -0.0400   -0.0550   -0.0700
% Yrand=[1.041:-0.015:0.906];     %  1.0410    1.0260    1.0110    0.9960    0.9810    0.9660    0.9510    0.9360    0.9210    0.9060
%                                 %    0.8910    0.8760    0.8610    0.8460    0.8310
%                                 
% Zrand=[0.3:0.1:1.9];                                
% Xu=0.13;    %  [0.32:-0.038:-0.326]=   0.3200    0.2820    0.2440    0.2060    0.1680    0.1300    0.0920    0.0540    0.0160   -0.0220   -0.0600   -0.0980
%             %                          -0.1360   -0.1740   -0.2120   -0.2500   -0.2880   -0.3260         
% %Xl=-0.08;   
% Yu=1.031;   %  [1.145:-0.038:0.803]= 1.1450    1.1070    1.0690    1.0310    0.9930    0.9550    0.9170    0.8790    0.8410    0.8030
% %Yl=0.82;    
% Z=0.5;  %   1.9000    1.8000    1.7000    1.6000    1.5000    1.4000    1.3000    1.2000    1.1000    1.0000
%         %   0.9000    0.8000    0.7000    0.6000    0.5000    0.4000    0.3000    0.2000
% X=Xu;
% Test motor values
testPans=[-20:0.1:+20]; 
testTilts=[-12:0.1:+12]; 
eps=0;
ePs=0;
pause(15);
f=0;
tic
%% Iterative test
%for a=1:15
for i=1:2
%     Y=Yu;
%     Zu=Z;
    for j=1:1
        %X=Xu(randi(length(Xu)));
        %Y=Yu(randi(length(Yu)));
%         objs.clear();
%         %objs.addString('world set sbox 1');
%         objs.addDouble(Xrand(randi(length(Xrand))));%
%         objs.addDouble(Yrand(randi(length(Yrand))));%
%         objs.addDouble(Zrand(randi(length(Zrand))));%
        %objcoorPort.open('/matlab/objcoordTx');
        % send the message
%         objcoorPort.write(objs);
        %disp('Sent Coordinates=');
        %disp(objs.toString_c());  
%         Zu=Zu-0.1;
%         Y=Y-0.015;  
        % pause(2);
       
        ydLeft=zeros(2,1);% 65 for logpolar and 60 for cartesian
        ydRight=zeros(2,1);% 65
        [returnCode]=vrep.simxSetObjectPosition(clientID,target,-1,[X,Y,Z],vrep.simx_opmode_oneshot_wait);
        d=0;
        p=0;   
        for k=1:1%length(allowedPans)
            for l=1:5%length(allowedTilts)
               
            
                panVal=allowedPans(randi(length(allowedPans)));%
                tiltVal=allowedTilts(randi(length(allowedTilts)));% 
                if l==4
                    panVal=allowedPans(1);
                    tiltVal=allowedTilts(1);
                end
                if l==5
                    panVal=allowedPans(41);
                    tiltVal=allowedTilts(1);
                end
               
%                 headPos.positionMove(4,panVal); % Joint 4 both Eyes pan
%                 headPos.positionMove(3,tiltVal); % Joint 3 both Eyes tilt
                [returnCode]=vrep.simxSetObjectPosition(clientID,left_eye,-1,lefteye_position+0.01.*[0,panVal,tiltVal],vrep.simx_opmode_oneshot_wait);
                [returnCode]=vrep.simxSetObjectPosition(clientID,right_eye,-1,righteye_position+0.01.*[0,panVal,tiltVal],vrep.simx_opmode_oneshot_wait);
                
                pause(3);
                %headEncs.getEncoders(encoders.data());
                %if (headDriver.isValid==0)
                %    disp('Cannot access to the Head controller')
                %    dbstop 164
                %end
%                 [retinaLeft]=retinaPreprocess(retinaSize,portLeft);
%                 [retinaRight]=retinaPreprocess(retinaSize,portRight); 
               
                [returnCode,resolut,image]= vrep.simxGetVisionSensorImage2(clientID,left_eye,0,vrep.simx_opmode_buffer); 
                [imageGray_left,image_left]=createRedMask(image);
                [retinaLeft]=imageGray_left;
                inputXL=image;
                
                [returnCode,resolut,image]= vrep.simxGetVisionSensorImage2(clientID,right_eye,0,vrep.simx_opmode_buffer); 
                [imageGray_right,image_right]=createRedMask(image);
                [retinaRight]=imageGray_right;




                if (sum(sum(retinaLeft)))>0
                    [ydLeft,indL,y]=eyeActivation(Wleft,Vleft,retinaFilters,retinaLeft,gaussCentersX,gaussCentersY,panVal,tiltVal);
                end
                if (sum(sum(retinaRight)))>0
                    [ydRight,indR]=eyeActivation(Wright,Vright,retinaFilters,retinaRight,gaussCentersX,gaussCentersY,panVal,tiltVal); 
                end
                rb=ydLeft;
                
                xGlobal=[ydLeft;ydRight];
                if sum(xGlobal)>0
                    %% Conjunction Activation  
                    XGlobal=[xGlobal;zeros(size(WGlobal,2)-size(xGlobal,1),1)];
                    [~,rG,~]=conj_act(WGlobal,VGlobal,XGlobal);
                    ydGlobal=rG(size(xGlobal,1)+1:end);
                    [~,indG]=max(ydGlobal);
                    %for kk=1:nd
                    %   disj_nodeRespMax{kk}(i,j)=max(disj_nodeRespMax{kk}(i,j),ydGlobal(kk));
                    %   disj_nodeRespMin{kk}(i,j)=min(disj_nodeRespMin{kk}(i,j),ydGlobal(kk));
                    %end 
                    xG=[zeros(size(xGlobal,1),1);ydGlobal];
                    [~,RG,~]=conj_act(WGlobal,VGlobal,xG);
                    XG=RG(1:size(xGlobal,1));
                    ydL=XG(1:length(XG)/2);
                    ydR=XG((length(XG)/2)+1:end);
                    [PanL,TiltL,y1]=eyeMotorCommands(Wleft,Vleft,length(retinaFilters),fovealGaussian,ydL,gaussCentersX,gaussCentersY);
                    [PanR,TiltR]=eyeMotorCommands(Wright,Vright,length(retinaFilters),fovealGaussian,ydR,gaussCentersX,gaussCentersY);
                    %% Headcentric Map formation                    
                    if ((sum(ydLeft)>0) && (sum(ydRight))>0)
                        panIndL=find(allowedPans==round(PanL));
                        tiltIndL=find(allowedTilts==round(TiltL));
                        headcentricMap(tiltIndL,panIndL)=indG;
                    else
                        if sum(ydLeft)>0 
                            panIndL=find(allowedPans==round(PanL));
                            tiltIndL=find(allowedTilts==round(TiltL));
                            headcentricMap(tiltIndL,panIndL)=indG;
                        end
                        if sum(ydRight)>0
                            panIndR=find(allowedPans==round(PanR+avgHorzDisp));
                            if isempty(panIndR) 
                                panDiff=round(PanR+avgHorzDisp)-max(allowedPans);
                                panIndR=length(allowedPans)+panDiff;
                            end
                            tiltIndR=find(allowedTilts==round(TiltR));
                            headcentricMap(tiltIndR,panIndR)=indG;
                        end
                    end
                    %% iCub motor commands                                                           
                   
%                     [Vg,Vs,Vt]=iCubMotorCommands(PanL,TiltL,PanR,TiltR);
%                     headPos.positionMove(5,Vg); % Joint 5 both Eyes vergence
%                     headPos.positionMove(4,Vs); % Joint 4 both Eyes pan/version
%                     headPos.positionMove(3,Vt); % Joint 3 both Eyes tilt
%                     pause(3);  
%                     [retinaLeft]=retinaPreprocess(retinaSize,portLeft);
%                     [retinaRight]=retinaPreprocess(retinaSize,portRight);
%                     retinaLeft=im2bw(retinaLeft);                    
%                    
                    
                     [returnCode]=vrep.simxSetObjectPosition(clientID,left_eye,-1,lefteye_position+0.01.*[0,PanL,TiltL],vrep.simx_opmode_oneshot_wait);
                     [returnCode]=vrep.simxSetObjectPosition(clientID,right_eye,-1,righteye_position+0.01.*[0,PanR,TiltR],vrep.simx_opmode_oneshot_wait);
                     pause(3);

                    [returnCode,resolut,image]= vrep.simxGetVisionSensorImage2(clientID,left_eye,0,vrep.simx_opmode_buffer); 
                    [imageGray_left,image_left]=createRedMask(image);
                    [retinaLeft]=imageGray_left;

                    [returnCode,resolut,image]= vrep.simxGetVisionSensorImage2(clientID,right_eye,0,vrep.simx_opmode_buffer); 
                    [imageGray_right,image_right]=createRedMask(image);
                    [retinaRight]=imageGray_right;
                    
                    
                    
                    
                    if (sum(retinaLeft(:))>0) && (sum(retinaRight(:))>0)
                        retRespL=retinal_resp(retinaFilters,retinaLeft);
                        retRespR=retinal_resp(retinaFilters,retinaRight);
                        f=f+1;
                        reponseLeft(f)=retRespL(fovealGaussian);
                        reponseRight(f)=retRespR(fovealGaussian);
%                         [L,num] = bwlabel(retinaLeft);
%                         stats = regionprops(L,retinaLeft,'Centroid');
%                         eps=eps+1;
%                         postSaccadicDstLeft(eps)=sqrt((64-round( stats.Centroid(1))).^2+(64-round( stats.Centroid(2))).^2);
%                         retinaRight=im2bw(retinaRight);
%                         [L,num] = bwlabel(retinaRight);
%                         stats = regionprops(L,retinaRight,'Centroid');
%                         postSaccadicDstRight(eps)=sqrt((64-round( stats.Centroid(1))).^2+(64-round( stats.Centroid(2))).^2);
                        %postSaccadicDst_without_CorrSacc(eps)=(postSaccadicDstLeft+postSaccadicDstRight)/2; 
                        if ((retRespL(fovealGaussian)>0.7) && (retRespR(fovealGaussian)>0.7))
                            
                        else
                            [ydLeft,indL]=eyeActivation(Wleft,Vleft,retinaFilters,retinaLeft,gaussCentersX,gaussCentersY,PanL,TiltL);
                            [ydRight,indR]=eyeActivation(Wright,Vright,retinaFilters,retinaRight,gaussCentersX,gaussCentersY,PanR,TiltR);
                            xGlobal=[ydLeft;ydRight];
                            XGlobal=[xGlobal;zeros(size(WGlobal,2)-size(xGlobal,1),1)];
                            [~,rG,~]=conj_act(WGlobal,VGlobal,XGlobal);
                            ydGlobal=rG(size(xGlobal,1)+1:end);
                            [~,indG]=max(ydGlobal);
                            xG=[zeros(size(xGlobal,1),1);ydGlobal];
                            [~,RG,~]=conj_act(WGlobal,VGlobal,xG);
                            XG=RG(1:size(xGlobal,1));
                            ydL=XG(1:length(XG)/2);
                            ydR=XG((length(XG)/2)+1:end);
                            [PanLCorrective,TiltLCorrective]=eyeMotorCommands(Wleft,Vleft,length(retinaFilters),fovealGaussian,ydL,gaussCentersX,gaussCentersY);
                            [PanRCorrective,TiltRCorrective]=eyeMotorCommands(Wright,Vright,length(retinaFilters),fovealGaussian,ydR,gaussCentersX,gaussCentersY);
                            % iCub motor commands                                                          
                           
%                             [Vg,Vs,Vt]=iCubMotorCommands(PanLCorrective,TiltLCorrective,PanRCorrective,TiltRCorrective);
%                             headPos.positionMove(5,Vg); % Joint 5 both Eyes vergence
%                             headPos.positionMove(4,Vs); % Joint 4 both Eyes pan/version
%                             headPos.positionMove(3,Vt); % Joint 3 both Eyes tilt
%                             pause(3);
                           
                            [returnCode]=vrep.simxSetObjectPosition(clientID,left_eye,-1,lefteye_position+0.01.*[0,PanLCorrective,TiltLCorrective],vrep.simx_opmode_oneshot_wait);
                            [returnCode]=vrep.simxSetObjectPosition(clientID,right_eye,-1,righteye_position+0.01.*[0,PanRCorrective,TiltRCorrective],vrep.simx_opmode_oneshot_wait);

                            pause(3);
                            
                            
%                             [retinaLeft]=retinaPreprocess(retinaSize,portLeft);
%                             [retinaRight]=retinaPreprocess(retinaSize,portRight); 
                            
                            
                            
                            
                            ePs=ePs+1;
                            
%                             [L,num] = bwlabel(retinaLeft);
%                             stats = regionprops(L,retinaLeft,'Centroid');
%                             postSaccadicDstLeftCorr(ePs)=sqrt((64-round( stats.Centroid(1))).^2+(64-round( stats.Centroid(2))).^2);
%                             retinaRight=im2bw(retinaRight);
%                             [L,num] = bwlabel(retinaRight);
%                             stats = regionprops(L,retinaRight,'Centroid');
%                             postSaccadicDstRightCorr(ePs)=sqrt((64-round( stats.Centroid(1))).^2+(64-round( stats.Centroid(2))).^2);
%                             postSaccadicDst_with_CorrSacc(ePs)=(postSaccadicDstLeft+postSaccadicDstRight)/2; 
                        end
                    end
                    %headEncs.getEncoders(encoders.data());
                    %disp('current position :');
                    %disp(encoders.toString_c());
                    %pause(0.1); 
                    %% Eyes Initial Position
                    
                    [returnCode]=vrep.simxSetObjectPosition(clientID,left_eye,-1,lefteye_position,vrep.simx_opmode_oneshot_wait);
                    [returnCode]=vrep.simxSetObjectPosition(clientID,right_eye,-1,righteye_position,vrep.simx_opmode_oneshot_wait);
                    
%                     headPos.positionMove(5,0);
%                     headPos.positionMove(4,0);
%                     headPos.positionMove(3,0);
                    
                    
                    %pause(3);
                end
            end
        end
    end
    X=X+0.3;   
end
%end
%receptiveFields(disj_nodeRespMax,0);
% mean_without_corrSac=sum(postSaccadicDst_without_CorrSacc)/length(postSaccadicDst_without_CorrSacc)
% Variance_without_corrSac=sum((mean_without_corrSac-postSaccadicDst_without_CorrSacc).^2)/length(postSaccadicDst_without_CorrSacc)
% SD_without_corrSac=sqrt(Variance_without_corrSac)
% mean_with_corrSac=sum(postSaccadicDst_with_CorrSacc)/length(postSaccadicDst_with_CorrSacc)
% Variance_with_corrSac=sum((mean_with_corrSac-postSaccadicDst_with_CorrSacc).^2)/length(postSaccadicDst_with_CorrSacc)
% SD_with_corrSac=sqrt(Variance_with_corrSac)
% mean_with_2corrSac=sum(postSaccadicDst_with_2CorrSacc)/length(postSaccadicDst_with_2CorrSacc)
% Variance_with_2corrSac=sum((mean_with_2corrSac-postSaccadicDst_with_2CorrSacc).^2)/length(postSaccadicDst_with_2CorrSacc)
% SD_with_2corrSac=sqrt(Variance_with_2corrSac)
toc


            vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait);
        % Now close the connection to V-REP:    
             vrep.simxFinish(clientID);
    else
        disp('Failed connecting to remote API server');
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Functions
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
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [Vg,Vs,Vt]=iCubMotorCommands(PanL,TiltL,PanR,TiltR)
% iCub motor commands 
Vg=PanL-PanR;
Vs=(PanL+PanR)/2;
Vt=(TiltL+TiltR)/2;