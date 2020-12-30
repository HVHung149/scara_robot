function varargout = hoang_viet_hung_1711626(varargin)
% HOANG_VIET_HUNG_1711626 MATLAB code for hoang_viet_hung_1711626.fig
%      HOANG_VIET_HUNG_1711626, by itself, creates dothia new HOANG_VIET_HUNG_1711626 or raises the existing
%      singleton*.
%
%      H = HOANG_VIET_HUNG_1711626 returns the handle to dothia new HOANG_VIET_HUNG_1711626 or the handle to
%      the existing singleton*.
%
%      HOANG_VIET_HUNG_1711626('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in HOANG_VIET_HUNG_1711626.M with the given input arguments.
%
%      HOANG_VIET_HUNG_1711626('Property','Value',...) creates dothia new HOANG_VIET_HUNG_1711626 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before hoang_viet_hung_1711626_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to hoang_viet_hung_1711626_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help hoang_viet_hung_1711626

% Last Modified by GUIDE v2.5 11-Dec-2020 08:36:36

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @hoang_viet_hung_1711626_OpeningFcn, ...
                   'gui_OutputFcn',  @hoang_viet_hung_1711626_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before hoang_viet_hung_1711626 is made visible.
function hoang_viet_hung_1711626_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to hoang_viet_hung_1711626 (see VARARGIN)
axes(handles.picture);
cla
matlabImage = imread('bachkhoa2.png');
image(matlabImage)
axis off
axis image
theta_1=0; theta_2=0; theta_4=0; d3=-50;
handles.dothia=[400 250 0 0]';
handles.alpha=[0 0 0 pi]';
handles.d=[383.5 0 d3 0]';
handles.theta=[theta_1 theta_2 0 theta_4]';
%init joint position
x0=0; y0=0; z0=0;
x1=0; y1=0; z1=383.5;
x2=400; y2=0; z2=383.5;
x3=650; y3=0; z3=383.5;
x4= 650; y4=0; z4=330;

handles.J1=[x1 y1 z1 1]';
handles.J2=[x2 y2 z2 1]';
handles.J3=[x3 y3 z3 1]';
handles.J4=[x4 y4 z4 1]';
% init grip position;

handles.grip1=[650 50 290 1]';
handles.grip2=[650 50 330 1]';
handles.grip3=[650 -50 330 1]';
handles.grip4=[650 -50 290 1]';
%init theta d
handles.theta_init=[0 0 0 0];
handles.d_init=[385.3 0 -50 0];
hold on;

%3dplot robot use method
axes(handles.axes1);
object_3d(handles.J1,handles.J2,handles.J3,handles.J4,handles.grip1,handles.grip2,handles.grip3,handles.grip4);
hold off;
% Choose default command line output for hoang_viet_hung_1711626
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes hoang_viet_hung_1711626 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = hoang_viet_hung_1711626_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in btn_F.
function btn_F_Callback(hObject, eventdata, handles)
axes(handles.axes1);
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
degree_to_rad_const=pi/180;
%get theta124 and d3 from edit text and update DH table
length=50;
handles.theta=[str2double(get(handles.theta_1, 'String'))*degree_to_rad_const str2double(get(handles.theta_2, 'String'))*degree_to_rad_const 0 str2double(get(handles.theta_4, 'String'))*degree_to_rad_const]';
handles.d(3)=str2double(get(handles.d3, 'String'));
d_d3=(handles.d(3)-handles.d_init(3))/length;
d_theta1 = (handles.theta(1)-handles.theta_init(1))/length;
d_theta2 = (handles.theta(2)-handles.theta_init(2))/length;
d_theta4 = (handles.theta(4)-handles.theta_init(3))/length;
d_theta=[d_theta1 d_theta2 0 d_theta4];
denta_d3= [0 0 d_d3 0];
list_post =handles.J4';
size=1;
for j=1:length
    handles.theta_init= handles.theta_init + d_theta
    handles.d_init=handles.d_init +denta_d3
    %DH matrix between frame
    DH=zeros(4,4,4);
    for i=1:4
        DH(:,:,i)=[cos(handles.theta_init(i)) -cos(handles.alpha(i))*sin(handles.theta_init(i)) sin(handles.alpha(i))*sin(handles.theta_init(i)) handles.dothia(i)*cos(handles.theta_init(i));
                   sin(handles.theta_init(i)) cos(handles.alpha(i))*cos(handles.theta_init(i)) -sin(handles.alpha(i))*cos(handles.theta_init(i)) handles.dothia(i)*sin(handles.theta_init(i));
                   0 sin(handles.alpha(i)) cos(handles.alpha(i)) handles.d_init(i);
                   0 0 0 1];
    end
    %new joint position
    handles.J1_new=handles.J1;
    handles.J2_new=DH(:,:,1)*[0 0 0 1]';
    handles.J3_new=DH(:,:,1)*DH(:,:,2)*[0 0 0 1]';
    handles.J4_new=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*[0 0 0 1]';
    list_post(size+1,:)=handles.J4_new';
    size =size +1;
    %new grip position
    handles.grip1_new=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 -50 50 1]';
    handles.grip2_new=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 -50 0 1]';
    handles.grip3_new=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 50 0 1]';
    handles.grip4_new=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 50 50 1]';
    object_3d(handles.J1_new,handles.J2_new,handles.J3_new,handles.J4_new,handles.grip1_new,handles.grip2_new,handles.grip3_new,handles.grip4_new);
    set(handles.x,'String',sprintf('%.2f',handles.J4_new(1)));
    set(handles.y,'String',sprintf('%.2f',handles.J4_new(2)));
    set(handles.z,'String',sprintf('%.2f',handles.J4_new(3)));
    T_matrix=(DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4));
    rotation_matrix=T_matrix(1:3,1:3);
    handles.p1=atan2(-rotation_matrix(3,1),sqrt(rotation_matrix(3,2)^2+rotation_matrix(3,3)^2));
    handles.y1=atan2(rotation_matrix(2,1)/cos(handles.p1),rotation_matrix(1,1)/cos(handles.p1));
    %handles.r1=atan2(rotation_matrix(3,2)/cos(handles.p1),rotation_matrix(3,3)/cos(handles.p1));
    %set(handles.pick,'String',sprintf('%.2f',(handles.p1)/degree_to_rad_const));
    %set(handles.roll,'String',sprintf('%.2f',(handles.r1)/degree_to_rad_const));
    set(handles.yall,'String',sprintf('%.2f',(handles.y1)/degree_to_rad_const));
    
    if (j == length)
        % updata Join
        handles.J1= handles.J1_new;
        handles.J2= handles.J2_new;
        handles.J3= handles.J3_new;
        handles.J4= handles.J4_new;
        % update grip position;
        handles.grip1=handles.grip1_new;
        handles.grip2=handles.grip2_new;
        handles.grip3=handles.grip3_new;
        handles.grip4=handles.grip4_new;
        % calculate J4 axis
        J4_xaxis=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[300 0 0 1]';
        J4_yaxis=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 300 0 1]';
        J4_zaxis=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 0 100 1]';
        hold on;
        arrow3d([handles.J4_new(1) J4_xaxis(1)],[handles.J4_new(2) J4_xaxis(2)],[handles.J4_new(3) J4_xaxis(3)],0.5,3,5,'r');
        arrow3d([handles.J4_new(1) J4_yaxis(1)],[handles.J4_new(2) J4_yaxis(2)],[handles.J4_new(3) J4_yaxis(3)],0.5,3,5,'b');
        arrow3d([handles.J4_new(1) J4_zaxis(1)],[handles.J4_new(2) J4_zaxis(2)],[handles.J4_new(3) J4_zaxis(3)],0.5,9,15,'g');
        %calculate J3 axis
        J3_xaxis=DH(:,:,1)*DH(:,:,2)*[300 0 0 1]';
        J3_yaxis=DH(:,:,1)*DH(:,:,2)*[0 300 0 1]';
        J3_zaxis=DH(:,:,1)*DH(:,:,2)*[0 0 100 1]';
        arrow3d([handles.J3_new(1) J3_xaxis(1)],[handles.J3_new(2) J3_xaxis(2)],[handles.J3_new(3) J3_xaxis(3)],0.5,3,5,'r');
        arrow3d([handles.J3_new(1) J3_yaxis(1)],[handles.J3_new(2) J3_yaxis(2)],[handles.J3_new(3) J3_yaxis(3)],0.5,3,5,'b');
        arrow3d([handles.J3_new(1) J3_zaxis(1)],[handles.J3_new(2) J3_zaxis(2)],[handles.J3_new(3) J3_zaxis(3)],0.5,9,15,'g');
        %calculate J2 axis
        J2_xaxis=DH(:,:,1)*[300 0 0 1]';
        J2_yaxis=DH(:,:,1)*[0 300 0 1]';
        J2_zaxis=DH(:,:,1)*[0 0 100 1]';
        arrow3d([handles.J2_new(1) J2_xaxis(1)],[handles.J2_new(2) J2_xaxis(2)],[handles.J2_new(3) J2_xaxis(3)],0.5,3,5,'r');
        arrow3d([handles.J2_new(1) J2_yaxis(1)],[handles.J2_new(2) J2_yaxis(2)],[handles.J2_new(3) J2_yaxis(3)],0.5,3,5,'b');
        arrow3d([handles.J2_new(1) J2_zaxis(1)],[handles.J2_new(2) J2_zaxis(2)],[handles.J2_new(3) J2_zaxis(3)],0.5,9,15,'g');
        %calculate frame 0 axis
        frame0_xaxis=[300 0 0 1]';
        frame0_yaxis=[0 300 0 1]';
        frame0_zaxis=[0 0 100 1]';
        arrow3d([0 frame0_xaxis(1)],[0 frame0_xaxis(2)],[0 frame0_xaxis(3)],0.5,3,5,'r');
        arrow3d([0 frame0_yaxis(1)],[0 frame0_yaxis(2)],[0 frame0_yaxis(3)],0.5,3,5,'b');
        arrow3d([0 frame0_zaxis(1)],[0 frame0_zaxis(2)],[0 frame0_zaxis(3)],0.5,9,15,'g');
    end
 delay_ms(1);
  
end
planning(list_post,size);
hold off;

% --- Executes on button press in btn_I.
function btn_I_Callback(hObject, eventdata, handles)
axes(handles.axes1);
% hObject    handle to btn_I (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%syms x=0;
length=50;
degree_to_rad_const = pi/180;
handles.coordinate=[str2double(get(handles.x, 'String')) str2double(get(handles.y, 'String')) str2double(get(handles.z, 'String'))]';
handles.coordinate_init=[0 0 0 ]';
handles.rpy=       [str2double(get(handles.roll,'String')) str2double(get(handles.pick,'String')) str2double(get(handles.yall,'String'))]';
c2=(handles.coordinate(1)^2+handles.coordinate(2)^2-handles.dothia(1)^2-handles.dothia(2)^2)/(2* handles.dothia(1)*handles.dothia(2));
s2 =sqrt(1-c2^2);
handles.theta(2)=atan2(s2,c2);
s1=((handles.dothia(1)+handles.dothia(2)*c2)*handles.coordinate(2)-handles.dothia(2)*s2*handles.coordinate(1))/(handles.coordinate(1)^2+handles.coordinate(2)^2);
c1= ((handles.dothia(1)+handles.dothia(2)*c2)*handles.coordinate(1)+handles.dothia(2)*s2*handles.coordinate(2))/(handles.coordinate(1)^2+ handles.coordinate(2)^2);
handles.theta(1)= atan2(s1,c1);
handles.theta(4)= handles.rpy(3)*degree_to_rad_const- handles.theta(1)- handles.theta(2);
handles.d(3)= handles.coordinate(3) - 383.5;
d_d3=(handles.d(3)-handles.d_init(3))/length;

d_theta1 = (handles.theta(1)-handles.theta_init(1))/length;
d_theta2 = (handles.theta(2)-handles.theta_init(2))/length;
d_theta4 = (handles.theta(4)-handles.theta_init(3))/length;
d_theta=[d_theta1 d_theta2 0 d_theta4];
denta_d3= [0 0 d_d3 0];
list_post =handles.J4';
size=1;
for j=1:length
    handles.theta_init = handles.theta_init + d_theta;
    handles.d_init = handles.d_init +denta_d3;
    %DH matrix between frame
    DH=zeros(4,4,4);
    for i=1:4
        DH(:,:,i)=[cos(handles.theta_init(i)) -cos(handles.alpha(i))*sin(handles.theta_init(i)) sin(handles.alpha(i))*sin(handles.theta_init(i)) handles.dothia(i)*cos(handles.theta_init(i));
                   sin(handles.theta_init(i)) cos(handles.alpha(i))*cos(handles.theta_init(i)) -sin(handles.alpha(i))*cos(handles.theta_init(i)) handles.dothia(i)*sin(handles.theta_init(i));
                   0 sin(handles.alpha(i)) cos(handles.alpha(i)) handles.d_init(i);
                   0 0 0 1];
    end
    %new joint position
    handles.J1_new=handles.J1;
    handles.J2_new=DH(:,:,1)*[0 0 0 1]';
    handles.J3_new=DH(:,:,1)*DH(:,:,2)*[0 0 0 1]';
    handles.J4_new=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*[0 0 0 1]';
    list_post(size+1,:)=handles.J4_new';
    size =size +1;
    
    %new grip position
    handles.grip1_new=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 -50 50 1]';
    handles.grip2_new=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 -50 0 1]';
    handles.grip3_new=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 50 0 1]';
    handles.grip4_new=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 50 50 1]';
    object_3d(handles.J1_new,handles.J2_new,handles.J3_new,handles.J4_new,handles.grip1_new,handles.grip2_new,handles.grip3_new,handles.grip4_new);
    T_matrix=(DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4));
    rotation_matrix=T_matrix(1:3,1:3);
     handles.p1=atan2(-rotation_matrix(3,1),sqrt(rotation_matrix(3,2)^2+rotation_matrix(3,3)^2));
    handles.y1=atan2(rotation_matrix(2,1)/cos(handles.p1),rotation_matrix(1,1)/cos(handles.p1));
    handles.r1=atan2(rotation_matrix(3,2)/cos(handles.p1),rotation_matrix(3,3)/cos(handles.p1));
    
    set(handles.theta_1,'String',sprintf('%.2f',handles.theta_init(1)/degree_to_rad_const));
    set(handles.theta_2,'String',sprintf('%.2f',handles.theta_init(2)/degree_to_rad_const));
    set(handles.theta_4,'String',sprintf('%.2f',handles.theta_init(4)/degree_to_rad_const));
    set(handles.d3,'String',sprintf('%.2f', handles.d_init(3)));
    if (j == length)
        % updata Join
        handles.J1= handles.J1_new;
        handles.J2= handles.J2_new;
        handles.J3= handles.J3_new;
        handles.J4= handles.J4_new;
        % update grip position;
        handles.grip1=handles.grip1_new;
        handles.grip2=handles.grip2_new;
        handles.grip3=handles.grip3_new;
        handles.grip4=handles.grip4_new;
        % calculate J4 axis
        J4_xaxis=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[300 0 0 1]';
        J4_yaxis=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 300 0 1]';
        J4_zaxis=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 0 100 1]';
        hold on;
        arrow3d([handles.J4_new(1) J4_xaxis(1)],[handles.J4_new(2) J4_xaxis(2)],[handles.J4_new(3) J4_xaxis(3)],0.5,3,5,'r');
        arrow3d([handles.J4_new(1) J4_yaxis(1)],[handles.J4_new(2) J4_yaxis(2)],[handles.J4_new(3) J4_yaxis(3)],0.5,3,5,'b');
        arrow3d([handles.J4_new(1) J4_zaxis(1)],[handles.J4_new(2) J4_zaxis(2)],[handles.J4_new(3) J4_zaxis(3)],0.5,9,15,'g');
        %calculate J3 axis
        J3_xaxis=DH(:,:,1)*DH(:,:,2)*[300 0 0 1]';
        J3_yaxis=DH(:,:,1)*DH(:,:,2)*[0 300 0 1]';
        J3_zaxis=DH(:,:,1)*DH(:,:,2)*[0 0 100 1]';
        arrow3d([handles.J3_new(1) J3_xaxis(1)],[handles.J3_new(2) J3_xaxis(2)],[handles.J3_new(3) J3_xaxis(3)],0.5,3,5,'r');
        arrow3d([handles.J3_new(1) J3_yaxis(1)],[handles.J3_new(2) J3_yaxis(2)],[handles.J3_new(3) J3_yaxis(3)],0.5,3,5,'b');
        arrow3d([handles.J3_new(1) J3_zaxis(1)],[handles.J3_new(2) J3_zaxis(2)],[handles.J3_new(3) J3_zaxis(3)],0.5,9,15,'g');
        %calculate J2 axis
        J2_xaxis=DH(:,:,1)*[300 0 0 1]';
        J2_yaxis=DH(:,:,1)*[0 300 0 1]';
        J2_zaxis=DH(:,:,1)*[0 0 100 1]';
        arrow3d([handles.J2_new(1) J2_xaxis(1)],[handles.J2_new(2) J2_xaxis(2)],[handles.J2_new(3) J2_xaxis(3)],0.5,3,5,'r');
        arrow3d([handles.J2_new(1) J2_yaxis(1)],[handles.J2_new(2) J2_yaxis(2)],[handles.J2_new(3) J2_yaxis(3)],0.5,3,5,'b');
        arrow3d([handles.J2_new(1) J2_zaxis(1)],[handles.J2_new(2) J2_zaxis(2)],[handles.J2_new(3) J2_zaxis(3)],0.5,9,15,'g');
        %calculate frame 0 axis
        frame0_xaxis=[300 0 0 1]';
        frame0_yaxis=[0 300 0 1]';
        frame0_zaxis=[0 0 100 1]';
        arrow3d([0 frame0_xaxis(1)],[0 frame0_xaxis(2)],[0 frame0_xaxis(3)],0.5,3,5,'r');
        arrow3d([0 frame0_yaxis(1)],[0 frame0_yaxis(2)],[0 frame0_yaxis(3)],0.5,3,5,'b');
        arrow3d([0 frame0_zaxis(1)],[0 frame0_zaxis(2)],[0 frame0_zaxis(3)],0.5,9,15,'g');
    end
 delay_ms(1);
end
 planning(list_post,size);
 hold off;
function theta_1_Callback(hObject, eventdata, handles)
% hObject    handle to theta_1 (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_1 as text
%        str2double(get(hObject,'String')) returns contents of theta_1 as dothia double


% --- Executes during object creation, after setting all properties.
function theta_1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_1 (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have dothia white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta_2_Callback(hObject, eventdata, handles)
% hObject    handle to theta_2 (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_2 as text
%        str2double(get(hObject,'String')) returns contents of theta_2 as dothia double


% --- Executes during object creation, after setting all properties.
function theta_2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_2 (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have dothia white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function theta_4_Callback(hObject, eventdata, handles)
% hObject    handle to theta_4 (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of theta_4 as text
%        str2double(get(hObject,'String')) returns contents of theta_4 as dothia double


% --- Executes during object creation, after setting all properties.
function theta_4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to theta_4 (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have dothia white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function d3_Callback(hObject, eventdata, handles)
% hObject    handle to d3 (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of d3 as text
%        str2double(get(hObject,'String')) returns contents of d3 as dothia double


% --- Executes during object creation, after setting all properties.
function d3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to d3 (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have dothia white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function x_Callback(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x as text
%        str2double(get(hObject,'String')) returns contents of x as dothia double


% --- Executes during object creation, after setting all properties.
function x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have dothia white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y_Callback(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y as text
%        str2double(get(hObject,'String')) returns contents of y as dothia double


% --- Executes during object creation, after setting all properties.
function y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have dothia white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function z_Callback(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of z as text
%        str2double(get(hObject,'String')) returns contents of z as dothia double

% --- Executes during object creation, after setting all properties.
function z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to z (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have dothia white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function roll_Callback(hObject, eventdata, handles)
% hObject    handle to roll (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of roll as text
%        str2double(get(hObject,'String')) returns contents of roll as dothia double


% --- Executes during object creation, after setting all properties.
function roll_CreateFcn(hObject, eventdata, handles)
% hObject    handle to roll (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have dothia white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function pick_Callback(hObject, eventdata, handles)
% hObject    handle to pick (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of pick as text
%        str2double(get(hObject,'String')) returns contents of pick as dothia double


% --- Executes during object creation, after setting all properties.
function pick_CreateFcn(hObject, eventdata, handles)
% hObject    handle to pick (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have dothia white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yall_Callback(hObject, eventdata, handles)
% hObject    handle to yall (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yall as text
%        str2double(get(hObject,'String')) returns contents of yall as dothia double


% --- Executes during object creation, after setting all properties.
function yall_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yall (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have dothia white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --------------------------------------------------------------------
function Untitled_5_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_5 (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_2_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_2 (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_3_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_3 (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function Untitled_4_Callback(hObject, eventdata, handles)
% hObject    handle to Untitled_4 (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.qva=[str2double(get(handles.qmax, 'String')) str2double(get(handles.vmax, 'String')) str2double(get(handles.amax, 'String'))]';
handles.t(2) = (2*handles.qva(2))/handles.qva(3);  %tc
handles.t(1)= handles.t(2)/2;
handles.t(5) = handles.qva(1)/handles.qva(2) + handles.t(2);
handles.t(3) = handles.t(5) - handles.t(2);
handles.t(4) = (handles.t(5) - handles.t(2)/2);
handles.t_unit = handles.t(5)/1000;
handles.y3 =zeros(1000,1);
x=zeros(1000,1);

for i = 1:1000
    if i<1000
    x(i+1) = x(i) + handles.t_unit;
    end
    
    if  x(i) < handles.t(1)
        y1(i) = 2*(handles.qva(3)/handles.t(2))*x(i);
    elseif x(i) < handles.t(2)
        y1(i) = -2*(handles.qva(3)/handles.t(2))*x(i)+2*handles.qva(3) ;
    elseif x(i)< handles.t(3)
        y1(i) = 0;
    elseif x(i)< handles.t(4)
        y1(i) = -2*(handles.qva(3)/handles.t(2))*x(i) +2*(handles.qva(3)/handles.t(2))*(handles.t(5)-handles.t(2));
    elseif x(i)< handles.t(5)
        y1(i) = 2*(handles.qva(3)/handles.t(2))*x(i)-2*(handles.qva(3)/handles.t(2))*handles.t(5);
    end
    
    if x(i) < handles.t(1)
        y2(i) = (handles.qva(3)/handles.t(2))*x(i)^2;
    elseif x(i) < handles.t(2)
        y2(i) = -(handles.qva(3)/handles.t(2))*x(i)^2+2*handles.qva(3)*x(i)-handles.qva(2);
    elseif x(i)< handles.t(3)
        y2(i) = handles.qva(2);
    elseif x(i)< handles.t(4)
        y2(i) =  -(handles.qva(3)/handles.t(2))*x(i)^2 +2*(handles.qva(3)/handles.t(2))*(handles.t(5)-handles.t(2))*x(i)+...
                    handles.qva(2)-(handles.qva(3)/handles.t(2))*((handles.t(5)-handles.t(2))^2);
    elseif x(i)< handles.t(5)
        y2(i) = (handles.qva(3)/handles.t(2))*x(i)^2-2*(handles.qva(3)/handles.t(2))*handles.t(5)*x(i)+...
                    (handles.qva(3)/handles.t(2))*handles.t(5)^2;
    end
     
    if x(i) < handles.t(1)
         y3(i) = (1/3)*(handles.qva(3)/handles.t(2))*x(i)^3;
    elseif x(i) < handles.t(2)
         y3(i) = -(1/3)*(handles.qva(3)/handles.t(2))*x(i)^3 + handles.qva(3)*x(i)^2 - handles.qva(2)*x(i)-...
                    (1/6)*(handles.qva(3)*handles.t(2)^2)+handles.qva(2)*(handles.t(2)/2);
    elseif x(i)< handles.t(3)
         y3(i) =handles.qva(3)*handles.t(2)*x(i)/2-handles.qva(3)*handles.t(2)^2/4;
    elseif x(i)< handles.t(4)
        y3(i) = -handles.qva(3)*(x(i)-handles.t(5)+handles.t(2))^3/3/handles.t(2) + handles.qva(3)*handles.t(2)*x(i)/2 -handles.qva(3)*handles.t(2)^2/4;
    elseif x(i)< handles.t(5)
        y3(i) = handles.qva(3)*(x(i)-handles.t(5))^3/3/handles.t(2)+handles.qva(3)*handles.t(2)*(handles.t(5)-handles.t(2))/2;
    end
end
% axes(handles.axes1);
% hold off;
axes(handles.dothiv);
hold off;
plot(x,y2,'g');
grid on;
axes(handles.dothiq);
hold off;
plot(x,y3,'b');
grid on;
axes(handles.axes9);
hold off;
plot(x,y1,'r');
grid on;

function amax_Callback(hObject, eventdata, handles)
% hObject    handle to amax (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of amax as text
%        str2double(get(hObject,'String')) returns contents of amax as dothia double


% --- Executes during object creation, after setting all properties.
function amax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to amax (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have dothia white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function vmax_Callback(hObject, eventdata, handles)
% hObject    handle to vmax (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of vmax as text
%        str2double(get(hObject,'String')) returns contents of vmax as dothia double


% --- Executes during object creation, after setting all properties.
function vmax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to vmax (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have dothia white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function qmax_Callback(hObject, eventdata, handles)
% hObject    handle to qmax (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of qmax as text
%        str2double(get(hObject,'String')) returns contents of qmax as dothia double


% --- Executes during object creation, after setting all properties.
function qmax_CreateFcn(hObject, eventdata, handles)
% hObject    handle to qmax (see GCBO)
% eventdata  reserved - to be defined in dothia future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have dothia white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function x_tool_Callback(hObject, eventdata, handles)
% hObject    handle to x_tool (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of x_tool as text
%        str2double(get(hObject,'String')) returns contents of x_tool as a double


% --- Executes during object creation, after setting all properties.
function x_tool_CreateFcn(hObject, eventdata, handles)
% hObject    handle to x_tool (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function y_tool_Callback(hObject, eventdata, handles)
% hObject    handle to y_tool (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of y_tool as text
%        str2double(get(hObject,'String')) returns contents of y_tool as a double


% --- Executes during object creation, after setting all properties.
function y_tool_CreateFcn(hObject, eventdata, handles)
% hObject    handle to y_tool (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function z_tool_Callback(hObject, eventdata, handles)
% hObject    handle to z_tool (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of z_tool as text
%        str2double(get(hObject,'String')) returns contents of z_tool as a double


% --- Executes during object creation, after setting all properties.
function z_tool_CreateFcn(hObject, eventdata, handles)
% hObject    handle to z_tool (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yall_tool_Callback(hObject, eventdata, handles)
% hObject    handle to yall_tool (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yall_tool as text
%        str2double(get(hObject,'String')) returns contents of yall_tool as a double


% --- Executes during object creation, after setting all properties.
function yall_tool_CreateFcn(hObject, eventdata, handles)
% hObject    handle to yall_tool (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_linear.
function btn_linear_Callback(hObject, eventdata, handles)
% hObject    handle to btn_linear (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
length=50;
x_linear = str2double(get(handles.x_tool, 'String'));
y_linear = str2double(get(handles.y_tool, 'String'));
z_linear = str2double(get(handles.z_tool, 'String'));
denta =(375-z_linear)/length;
z_default = 375;
yall_angle=str2double(get(handles.yall_tool,'String'));
qmax= sqrt((x_linear-650)^2+y_linear^2)
amax=10;
vmax=10;
     set(handles.qmax,'String',sprintf('%.2f',qmax));
     set(handles.vmax,'String',sprintf('%.2f', vmax));
     set(handles.amax,'String',sprintf('%.2f',amax));
t(2) = (2*vmax/amax);  %tc
t(1)=   t(2)/2;
t(5) = qmax/vmax + t(2);
t(3) = t(5) - t(2);
t(4) = t(5) - t(2)/2;
t_unit = t(5)/length;
x=zeros(length,1);
alpha_angle=atan(y_linear/(650-x_linear));

for i = 1:length
    if i<length
    x(i+1) = x(i) +t_unit;
    end
     if  x(i) < t(1)
        y1(i) = 2*(amax/t(2))*x(i);
    elseif x(i) < t(2)
        y1(i) = -2*(amax/t(2))*x(i)+2*amax ;
    elseif x(i)< t(3)
        y1(i) = 0;
    elseif x(i)< t(4)
        y1(i) = -2*(amax/t(2))*x(i) +2*(amax/t(2))*(t(5)-t(2));
    elseif x(i)< t(5)
        y1(i) = 2*(amax/t(2))*x(i)-2*(amax/t(2))*t(5);
     end
    if x(i) < t(1)
        y2(i) = (amax/t(2))*x(i)^2;
    elseif x(i) < t(2)
        y2(i) = -(amax/t(2))*x(i)^2+2*amax*x(i)-vmax;
    elseif x(i)< t(3)
        y2(i) = vmax;
    elseif x(i)<t(4)
        y2(i) =  -(amax/t(2))*x(i)^2 +2*(amax/t(2))*(t(5)-t(2))*x(i)+...
                    vmax-(amax/t(2))*((t(5)-t(2))^2);
    elseif x(i)< t(5)
        y2(i) = (amax/t(2))*x(i)^2-2*(amax/t(2))*t(5)*x(i)+...
                    (amax/t(2))*t(5)^2;
    end
    if x(i) < t(1)
         y3(i) = (1/3)*(amax/t(2))*x(i)^3;
    elseif x(i) < t(2)
         y3(i) = -(1/3)*(amax/t(2))*x(i)^3 + amax*x(i)^2 - vmax*x(i)-...
                (1/6)*(amax*t(2)^2)+vmax*(t(2)/2);
    elseif x(i)< t(3)
         y3(i) =amax*t(2)*x(i)/2-amax*t(2)^2/4;
    elseif x(i)< t(4)
        y3(i) = -amax*(x(i)-t(5)+t(2))^3/3/t(2) + amax*t(2)*x(i)/2 -amax*t(2)^2/4;
    elseif x(i)< t(5)
        y3(i) = amax*(x(i)-t(5))^3/3/t(2)+amax*t(2)*(t(5)-t(2))/2;
    end
end
axes(handles.dothiv);
hold off;
plot(x,y2,'g');
grid on;
axes(handles.dothiq);
hold off;
plot(x,y3,'b');
grid on;
axes(handles.axes9);
hold off;
plot(x,y1,'r');
grid on;

x=zeros(length,1);
qx=650-caculate_qx(y3,alpha_angle);
qy=caculate_qy(y3,alpha_angle);
qx(length);
qy(length);
counter=0;
donext=0;
list_post =handles.J4';
for i = 1:length
    if i<length
    x(i+1) = x(i) + t_unit;
    end
    struct_inverse= inverse_skinematic(qx(i),qy(i),z_default,yall_angle,handles.dothia);
    z_default= z_default -denta;
    handles.theta(1)=struct_inverse(1);
    handles.theta(2)=struct_inverse(2);
    handles.d(3)= struct_inverse(3);
    handles.theta(4)=struct_inverse(4);
    flag= struct_inverse(5);
    if(flag==0)
    f = msgbox({'Invalid Value because over theta2 ';'Type Again'}, 'Error','error');
    donext=1;
    break;
    end
end
x=zeros(length,1);
z_default = 375;
if(donext==0)
    axes(handles.axes1);
    for i = 1:length
        if i<length
        x(i+1) = x(i) + t_unit;
        end
        struct_inverse= inverse_skinematic(qx(i),qy(i),z_default,yall_angle,handles.dothia);
        z_default= z_default -denta;
         handles.theta(1)=struct_inverse(1);
        handles.theta(2)=struct_inverse(2);
        handles.d(3)= struct_inverse(3);
        handles.theta(4)=struct_inverse(4);
        struct_forward =forward_skinematic(handles.theta,handles.alpha,handles.d,handles.dothia);
        list_post(counter+1,:)=struct_forward(4,:)';
        counter =counter +1;
        object_3d(struct_forward(1,:),struct_forward(2,:),struct_forward(3,:),struct_forward(4,:),struct_forward(5,:),struct_forward(6,:),struct_forward(7,:),struct_forward(8,:));
        if(i==length)
            hold on;
        for i=1:4
        DH(:,:,i)=[cos(handles.theta(i)) -cos(handles.alpha(i))*sin(handles.theta(i)) sin(handles.alpha(i))*sin(handles.theta(i)) handles.dothia(i)*cos(handles.theta(i));
               sin(handles.theta(i)) cos(handles.alpha(i))*cos(handles.theta(i)) -sin(handles.alpha(i))*cos(handles.theta(i)) handles.dothia(i)*sin(handles.theta(i));
               0 sin(handles.alpha(i)) cos(handles.alpha(i)) handles.d(i);
               0 0 0 1];
        end
        % calculate J4 axis
        J4_xaxis=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[300 0 0 1]';
        J4_yaxis=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 300 0 1]';
        J4_zaxis=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 0 100 1]';
        hold on;
        arrow3d([struct_forward(4,1) J4_xaxis(1)],[struct_forward(4,2) J4_xaxis(2)],[struct_forward(4,3) J4_xaxis(3)],0.5,3,5,'r');
        arrow3d([struct_forward(4,1) J4_yaxis(1)],[struct_forward(4,2) J4_yaxis(2)],[struct_forward(4,3) J4_yaxis(3)],0.5,3,5,'b');
        arrow3d([struct_forward(4,1) J4_zaxis(1)],[struct_forward(4,2) J4_zaxis(2)],[struct_forward(4,3) J4_zaxis(3)],0.5,9,15,'g');
        %calculate J3 axis
        J3_xaxis=DH(:,:,1)*DH(:,:,2)*[300 0 0 1]';
        J3_yaxis=DH(:,:,1)*DH(:,:,2)*[0 300 0 1]';
        J3_zaxis=DH(:,:,1)*DH(:,:,2)*[0 0 100 1]';
        arrow3d([struct_forward(3,1) J3_xaxis(1)],[struct_forward(3,2)  J3_xaxis(2)],[struct_forward(3,3) J3_xaxis(3)],0.5,3,5,'r');
        arrow3d([struct_forward(3,1)  J3_yaxis(1)],[struct_forward(3,2)  J3_yaxis(2)],[struct_forward(3,3) J3_yaxis(3)],0.5,3,5,'b');
        arrow3d([struct_forward(3,1)  J3_zaxis(1)],[struct_forward(3,2)  J3_zaxis(2)],[struct_forward(3,3) J3_zaxis(3)],0.5,9,15,'g');
        %calculate J2 axis
        J2_xaxis=DH(:,:,1)*[300 0 0 1]';
        J2_yaxis=DH(:,:,1)*[0 300 0 1]';
        J2_zaxis=DH(:,:,1)*[0 0 100 1]';
        arrow3d([struct_forward(2,1) J2_xaxis(1)],[struct_forward(2,2) J2_xaxis(2)],[struct_forward(2,3) J2_xaxis(3)],0.5,3,5,'r');
        arrow3d([struct_forward(2,1) J2_yaxis(1)],[struct_forward(2,2) J2_yaxis(2)],[struct_forward(2,3) J2_yaxis(3)],0.5,3,5,'b');
        arrow3d([struct_forward(2,1) J2_zaxis(1)],[struct_forward(2,2) J2_zaxis(2)],[struct_forward(2,3) J2_zaxis(3)],0.5,9,15,'g');
        %calculate frame 0 axis
        frame0_xaxis=[300 0 0 1]';
        frame0_yaxis=[0 300 0 1]';
        frame0_zaxis=[0 0 100 1]';
        arrow3d([0 frame0_xaxis(1)],[0 frame0_xaxis(2)],[0 frame0_xaxis(3)],0.5,3,5,'r');
        arrow3d([0 frame0_yaxis(1)],[0 frame0_yaxis(2)],[0 frame0_yaxis(3)],0.5,3,5,'b');
        arrow3d([0 frame0_zaxis(1)],[0 frame0_zaxis(2)],[0 frame0_zaxis(3)],0.5,9,15,'g');
        end
        delay_ms(1);
    end 
    planning(list_post,counter);
end




% --- Executes on button press in rdo_clockwise.
function rdo_clockwise_Callback(hObject, eventdata, handles)
% hObject    handle to rdo_clockwise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rdo_clockwise
handles.rdo_counter_clockwise.Value=0;
if(handles.rdo_clockwise.Value==1)
    handles.pos_tool=[str2double(get(handles.x_tool, 'String')) str2double(get(handles.y_tool, 'String')) str2double(get(handles.z_tool, 'String'))]';
    AB=sqrt((handles.pos_tool(1)-650)^2+(handles.pos_tool(2))^2);
    R=sprintf(' %.0f',AB/2)
    str=strcat('R radius must be rather than',R)
    f=msgbox(str);
end
% --- Executes on button press in rdo_counter_clockwise.
function rdo_counter_clockwise_Callback(hObject, eventdata, handles)
% hObject    handle to rdo_counter_clockwise (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of rdo_counter_clockwise
handles.rdo_clockwise.Value=0;
if(handles.rdo_counter_clockwise.Value==1)
    handles.pos_tool=[str2double(get(handles.x_tool, 'String')) str2double(get(handles.y_tool, 'String')) str2double(get(handles.z_tool, 'String'))]';
    AB=sqrt((handles.pos_tool(1)-650)^2+(handles.pos_tool(2))^2);
    R=sprintf(' %.0f',AB/2)
    str=strcat('R radius must be rather than',R)
    f=msgbox(str);
end
% --- Executes on button press in pushbutton7.
function pushbutton7_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
choose_C1=0;
choose_C2=0;
center_x=0;
center_y=0;
donext=1;
length=50;
r= str2double(get(handles.radius,'String'));
x_linear = str2double(get(handles.x_tool, 'String'));
y_linear = str2double(get(handles.y_tool, 'String'));
z_linear = str2double(get(handles.z_tool, 'String'));
yall_angle=str2double(get(handles.yall_tool,'String'));
pointA =[650 0];
pointB= [x_linear y_linear];
M =[(pointA(1)+pointB(1))/2 (pointA(2)+pointB(2))/2]; 
C= getCenter(pointA,pointB,r);
denta =(375-z_linear)/length;
z_default = 375;
if(C(1,1)<M(1))
    center_x=C(1,1)
    center_y=C(1,2)
else
    center_x=C(2,1)
    center_y=C(2,2)
end

%get AB
AB=sqrt((x_linear-650)^2+(y_linear)^2);
%set qmax
if(handles.rdo_clockwise.Value== true)
   if(y_linear>0)
       qmax =GetQmax(r,AB,false)
   else
       qmax =GetQmax(r,AB,true)
   end
end
if(handles.rdo_counter_clockwise.Value== true)
    if(y_linear>0)
       qmax =GetQmax(r,AB,true)
   else
       qmax =GetQmax(r,AB,false)
   end
end

amax=10;
vmax=30;
set(handles.qmax,'String',sprintf('%.2f',qmax));
set(handles.vmax,'String',sprintf('%.2f', vmax));
set(handles.amax,'String',sprintf('%.2f',amax));
t(2) = (2*vmax/amax);  %tc
t(1)=   t(2)/2;
t(5) = qmax/vmax + t(2);
t(3) = t(5) - t(2);
t(4) = t(5) - t(2)/2;
t_unit = t(5)/length;
x=zeros(length,1);
for i = 1:length
    if i<length
    x(i+1) = x(i) +t_unit;
    end
     if  x(i) < t(1)
        y1(i) = 2*(amax/t(2))*x(i);
    elseif x(i) < t(2)
        y1(i) = -2*(amax/t(2))*x(i)+2*amax ;
    elseif x(i)< t(3)
        y1(i) = 0;
    elseif x(i)< t(4)
        y1(i) = -2*(amax/t(2))*x(i) +2*(amax/t(2))*(t(5)-t(2));
    elseif x(i)< t(5)
        y1(i) = 2*(amax/t(2))*x(i)-2*(amax/t(2))*t(5);
     end
    if x(i) < t(1)
        y2(i) = (amax/t(2))*x(i)^2;
    elseif x(i) < t(2)
        y2(i) = -(amax/t(2))*x(i)^2+2*amax*x(i)-vmax;
    elseif x(i)< t(3)
        y2(i) = vmax;
    elseif x(i)<t(4)
        y2(i) =  -(amax/t(2))*x(i)^2 +2*(amax/t(2))*(t(5)-t(2))*x(i)+...
                    vmax-(amax/t(2))*((t(5)-t(2))^2);
    elseif x(i)< t(5)
        y2(i) = (amax/t(2))*x(i)^2-2*(amax/t(2))*t(5)*x(i)+...
                    (amax/t(2))*t(5)^2;
    end
    if x(i) < t(1)
         y3(i) = (1/3)*(amax/t(2))*x(i)^3;
    elseif x(i) < t(2)
         y3(i) = -(1/3)*(amax/t(2))*x(i)^3 + amax*x(i)^2 - vmax*x(i)-...
                (1/6)*(amax*t(2)^2)+vmax*(t(2)/2);
    elseif x(i)< t(3)
         y3(i) =amax*t(2)*x(i)/2-amax*t(2)^2/4;
    elseif x(i)< t(4)
        y3(i) = -amax*(x(i)-t(5)+t(2))^3/3/t(2) + amax*t(2)*x(i)/2 -amax*t(2)^2/4;
    elseif x(i)< t(5)
        y3(i) = amax*(x(i)-t(5))^3/3/t(2)+amax*t(2)*(t(5)-t(2))/2;
    end
end
axes(handles.dothiv);
hold off;
plot(x,y2,'g');
grid on;
axes(handles.dothiq);
hold off;
plot(x,y3,'b');
grid on;
axes(handles.axes9);
hold off;
plot(x,y1,'r');
grid on;
    if(center_y>=0)
    phi=-acos((650-center_x)/r)
    else
    phi=acos((650-center_x)/r)
    end
%phi=-(pi-acos((-650+C2(1))/r))
if(handles.rdo_clockwise.Value== true)
qx = get_qx_circle(-y3,r);
qy = get_qy_circle(-y3,r);
end
if(handles.rdo_counter_clockwise.Value== true)
qx = get_qx_circle(y3,r);
qy = get_qy_circle(y3,r);
end
DH= [cos(phi) -sin(phi) 0 center_x;...
    sin(phi) cos(phi) 0 center_y;...
    0 0 1 0;...
    0 0 0 1];
P=zeros(length,4);
for i = 1:length
    if i<length
    x(i+1) = x(i) +t_unit;
    end
   P(i,:)=(DH*[qx(i) qy(i) 0 1]')';
end
double(P)
if(P(1,1)==650 && abs(P(1,2))>=0 && abs(P(length,1)-x_linear)<=5 && abs(P(length,2)-y_linear)<=5)
    choose_C1=1;
    disp('accept C');
else
    disp('dont accept C1');
     f = msgbox({'nothing center accept';'Type Again R'}, 'Error','error');
end
if(choose_C1==1)
    for i = 1:length
        if i<length
        x(i+1) = x(i) + t_unit;
        end
        P(i,1);
        P(i,2);
        struct_inverse= inverse_skinematic(P(i,1),P(i,2),z_linear,yall_angle,handles.dothia);
        handles.theta(1)=struct_inverse(1);
        handles.theta(2)=struct_inverse(2);
        handles.d(3)= struct_inverse(3);
        handles.theta(4)=struct_inverse(4);
        flag_contrain_theta2= struct_inverse(5);
        flag_contrain_s2=struct_inverse(6);
        if(flag_contrain_theta2==0)
            donext=0; 
            f = msgbox({'Invalid Value because over theta2 ';'Type Again R'}, 'Error','error');
            break;
        elseif(flag_contrain_s2==0)
            donext=0;
            f = msgbox({'Invalid Value because over s2, theta2 ';'Type Again R'}, 'Error','error');
            break;
        end
    end
    if(donext)
        counter=0;
        axes(handles.axes1);
        list_post =handles.J4';
    for i = 1:length
        if i<length
        x(i+1) = x(i) + t_unit;
        end
         z_default= z_default -denta;
        struct_inverse= inverse_skinematic(P(i,1),P(i,2),z_default,yall_angle,handles.dothia);
        handles.theta(1)=struct_inverse(1);
        handles.theta(2)=struct_inverse(2);
        handles.d(3)= struct_inverse(3);
        handles.theta(4)=struct_inverse(4);
        struct_forward =forward_skinematic(handles.theta,handles.alpha,handles.d,handles.dothia);
        list_post(counter+1,:)=struct_forward(4,:)';
        counter =counter +1;
        object_3d(struct_forward(1,:),struct_forward(2,:),struct_forward(3,:),struct_forward(4,:),struct_forward(5,:),struct_forward(6,:),struct_forward(7,:),struct_forward(8,:));
        if(i==length)
            hold on;
        for i=1:4
        DH(:,:,i)=[cos(handles.theta(i)) -cos(handles.alpha(i))*sin(handles.theta(i)) sin(handles.alpha(i))*sin(handles.theta(i)) handles.dothia(i)*cos(handles.theta(i));
               sin(handles.theta(i)) cos(handles.alpha(i))*cos(handles.theta(i)) -sin(handles.alpha(i))*cos(handles.theta(i)) handles.dothia(i)*sin(handles.theta(i));
               0 sin(handles.alpha(i)) cos(handles.alpha(i)) handles.d(i);
               0 0 0 1];
        end
        % calculate J4 axis
        J4_xaxis=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[300 0 0 1]';
        J4_yaxis=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 300 0 1]';
        J4_zaxis=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 0 100 1]';
        hold on;
        arrow3d([struct_forward(4,1) J4_xaxis(1)],[struct_forward(4,2) J4_xaxis(2)],[struct_forward(4,3) J4_xaxis(3)],0.5,3,5,'r');
        arrow3d([struct_forward(4,1) J4_yaxis(1)],[struct_forward(4,2) J4_yaxis(2)],[struct_forward(4,3) J4_yaxis(3)],0.5,3,5,'b');
        arrow3d([struct_forward(4,1) J4_zaxis(1)],[struct_forward(4,2) J4_zaxis(2)],[struct_forward(4,3) J4_zaxis(3)],0.5,9,15,'g');
        %calculate J3 axis
        J3_xaxis=DH(:,:,1)*DH(:,:,2)*[300 0 0 1]';
        J3_yaxis=DH(:,:,1)*DH(:,:,2)*[0 300 0 1]';
        J3_zaxis=DH(:,:,1)*DH(:,:,2)*[0 0 100 1]';
        arrow3d([struct_forward(3,1) J3_xaxis(1)],[struct_forward(3,2)  J3_xaxis(2)],[struct_forward(3,3) J3_xaxis(3)],0.5,3,5,'r');
        arrow3d([struct_forward(3,1)  J3_yaxis(1)],[struct_forward(3,2)  J3_yaxis(2)],[struct_forward(3,3) J3_yaxis(3)],0.5,3,5,'b');
        arrow3d([struct_forward(3,1)  J3_zaxis(1)],[struct_forward(3,2)  J3_zaxis(2)],[struct_forward(3,3) J3_zaxis(3)],0.5,9,15,'g');
        %calculate J2 axis
        J2_xaxis=DH(:,:,1)*[300 0 0 1]';
        J2_yaxis=DH(:,:,1)*[0 300 0 1]';
        J2_zaxis=DH(:,:,1)*[0 0 100 1]';
        arrow3d([struct_forward(2,1) J2_xaxis(1)],[struct_forward(2,2) J2_xaxis(2)],[struct_forward(2,3) J2_xaxis(3)],0.5,3,5,'r');
        arrow3d([struct_forward(2,1) J2_yaxis(1)],[struct_forward(2,2) J2_yaxis(2)],[struct_forward(2,3) J2_yaxis(3)],0.5,3,5,'b');
        arrow3d([struct_forward(2,1) J2_zaxis(1)],[struct_forward(2,2) J2_zaxis(2)],[struct_forward(2,3) J2_zaxis(3)],0.5,9,15,'g');
        %calculate frame 0 axis
        frame0_xaxis=[300 0 0 1]';
        frame0_yaxis=[0 300 0 1]';
        frame0_zaxis=[0 0 100 1]';
        arrow3d([0 frame0_xaxis(1)],[0 frame0_xaxis(2)],[0 frame0_xaxis(3)],0.5,3,5,'r');
        arrow3d([0 frame0_yaxis(1)],[0 frame0_yaxis(2)],[0 frame0_yaxis(3)],0.5,3,5,'b');
        arrow3d([0 frame0_zaxis(1)],[0 frame0_zaxis(2)],[0 frame0_zaxis(3)],0.5,9,15,'g');
        end
        delay_ms(1);
    end
    planning(list_post,counter);  
    end
end

function radius_Callback(hObject, eventdata, handles)
% hObject    handle to radius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of radius as text
%        str2double(get(hObject,'String')) returns contents of radius as a double


% --- Executes during object creation, after setting all properties.
function radius_CreateFcn(hObject, eventdata, handles)
% hObject    handle to radius (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
% dong hoc thuan dong hoc nguoc trajectorry jacobi
function qy=caculate_qy(qt,alpha)
qy=qt*sin(alpha);

function qx=caculate_qx(qt,alpha)
qx=qt*cos(alpha);

function [h]=arrow3d(x,y,z,head_frac,radii,radii2,colr)
%
% The function plotting 3-dimensional arrow
%
% h=arrow3d(x,y,z,head_frac,radii,radii2,colr)
%
% The inputs are:
%       x,y,z =  vectors of the starting point and the ending point of the
%           arrow, e.g.:  x=[x_start, x_end]; y=[y_start, y_end];z=[z_start,z_end];
%       head_frac = fraction of the arrow length where the head should  start
%       radii = radius of the arrow
%       radii2 = radius of the arrow head (defult = radii*2)
%       colr =   color of the arrow, can be string of the color name, or RGB vector  (default='blue')
%
% The output is the handle of the surfaceplot graphics object.
% The settings of the plot can changed using: set(h, 'PropertyName', PropertyValue)
%
% example #1:
%        arrow3d([0 0],[0 0],[0 6],.5,3,4,[1 0 .5]);
% example #2:
%        arrow3d([2 0],[5 0],[0 -6],.2,3,5,'r');
% example #3:
%        h = arrow3d([1 0],[0 1],[-2 3],.8,3);
%        set(h,'facecolor',[1 0 0])
% 
% Written by Moshe Lindner , Bar-Ilan University, Israel.
% July 2010 (C)
if nargin==5
    radii2=radii*2;
    colr='blue';
elseif nargin==6
    colr='blue';
end
if size(x,1)==2
    x=x';
    y=y';
    z=z';
end
x(3)=x(2);
x(2)=x(1)+head_frac*(x(3)-x(1));
y(3)=y(2);
y(2)=y(1)+head_frac*(y(3)-y(1));
z(3)=z(2);
z(2)=z(1)+head_frac*(z(3)-z(1));
r=[x(1:2)',y(1:2)',z(1:2)'];
N=50;
dr=diff(r);
dr(end+1,:)=dr(end,:);
origin_shift=(ones(size(r))*(1+max(abs(r(:))))+[dr(:,1) 2*dr(:,2) -dr(:,3)]);
r=r+origin_shift;
normdr=(sqrt((dr(:,1).^2)+(dr(:,2).^2)+(dr(:,3).^2)));
normdr=[normdr,normdr,normdr];
dr=dr./normdr;
Pc=r;
n1=cross(dr,Pc);
normn1=(sqrt((n1(:,1).^2)+(n1(:,2).^2)+(n1(:,3).^2)));
normn1=[normn1,normn1,normn1];
n1=n1./normn1;
P1=n1+Pc;
X1=[];Y1=[];Z1=[];
j=1;
for theta=([0:N])*2*pi./(N);
    R1=Pc+radii*cos(theta).*(P1-Pc) + radii*sin(theta).*cross(dr,(P1-Pc)) -origin_shift;
    X1(2:3,j)=R1(:,1);
    Y1(2:3,j)=R1(:,2);
    Z1(2:3,j)=R1(:,3);
    j=j+1;
end
r=[x(2:3)',y(2:3)',z(2:3)'];
dr=diff(r);
dr(end+1,:)=dr(end,:);
origin_shift=(ones(size(r))*(1+max(abs(r(:))))+[dr(:,1) 2*dr(:,2) -dr(:,3)]);
r=r+origin_shift;
normdr=(sqrt((dr(:,1).^2)+(dr(:,2).^2)+(dr(:,3).^2)));
normdr=[normdr,normdr,normdr];
dr=dr./normdr;
Pc=r;
n1=cross(dr,Pc);
normn1=(sqrt((n1(:,1).^2)+(n1(:,2).^2)+(n1(:,3).^2)));
normn1=[normn1,normn1,normn1];
n1=n1./normn1;
P1=n1+Pc;
j=1;
for theta=([0:N])*2*pi./(N);
    R1=Pc+radii2*cos(theta).*(P1-Pc) + radii2*sin(theta).*cross(dr,(P1-Pc)) -origin_shift;
    X1(4:5,j)=R1(:,1);
    Y1(4:5,j)=R1(:,2);
    Z1(4:5,j)=R1(:,3);
    j=j+1;
end
X1(1,:)=X1(1,:)*0 + x(1);
Y1(1,:)=Y1(1,:)*0 + y(1);
Z1(1,:)=Z1(1,:)*0 + z(1);
X1(5,:)=X1(5,:)*0 + x(3);
Y1(5,:)=Y1(5,:)*0 + y(3);
Z1(5,:)=Z1(5,:)*0 + z(3);
h=surf(X1,Y1,Z1,'facecolor',colr,'edgecolor','none');
camlight
lighting phong

function delay_ms(n)
for i=1:n
   pause(0.0001);
end
function object_3d(arg1,arg2,arg3,arg4,arg5,arg6,arg7,arg8)
%with arg1234 is joint array ref to base frame
%arg5678 is grip array ref to base frame
%init plot

hold off;
%draw line
plot3([0 arg1(1)],[0 arg1(2)],[0 arg1(3)],'b','LineWidth',5);
hold on;
plot3([arg2(1) arg1(1)],[arg2(2) arg1(2)],[arg2(3) arg1(3)],'r','LineWidth',5);
plot3([arg3(1) arg2(1)],[arg3(2) arg2(2)],[arg3(3) arg2(3)],'y','LineWidth',5);
plot3([arg4(1) arg3(1)],[arg4(2) arg3(2)],[arg4(3) arg3(3)],'k','LineWidth',5);
denta=80;
denta_1=5;
%plot joint
%plot3(0,0,0,'o','MarkerSize',15,'MarkerEdgeColor','w','MarkerFaceColor','k');
[x1,y1,z1]= cylinder(denta);
x1=arg1(1)+x1;
y1=arg1(2)+y1;
z1=z1*10+arg1(3)-denta_1;
surf(x1,y1,z1,'facecolor','y','LineStyle','none');
fill3(x1(1,:),y1(1,:),z1(1,:),'y');
fill3(x1(2,:),y1(2,:),z1(2,:),'y');
%plot3(arg1(1),arg1(2),arg1(3),'o','MarkerSize',15,'MarkerEdgeColor','w','MarkerFaceColor','k');
[x2,y2,z2]= cylinder(denta);
x2=arg2(1)+x2;
y2=arg2(2)+y2;
z2=arg2(3)+z2*10-denta_1;
surf(x2,y2,z2,'facecolor','b','LineStyle','none');
fill3(x2(1,:),y2(1,:),z2(1,:),'b');
fill3(x2(2,:),y2(2,:),z2(2,:),'b');
%plot3(arg2(1),arg2(2),arg2(3),'o','MarkerSize',15,'MarkerEdgeColor','w','MarkerFaceColor','k');
[x3,y3,z3]= cylinder(denta);
x3=arg3(1)+x3;
y3=arg3(2)+y3;
z3=arg3(3)+z3*10-denta_1;
surf(x3,y3,z3,'facecolor','r','LineStyle','none');
fill3(x3(1,:),y3(1,:),z3(1,:),'r');
fill3(x3(2,:),y3(2,:),z3(2,:),'r');

%plot3(arg3(1),arg3(2),arg3(3),'o','MarkerSize',15,'MarkerEdgeColor','w','MarkerFaceColor','k');
[x4,y4,z4]= cylinder(denta);
x4=arg4(1)+x4;
y4=arg4(2)+y4;
z4=arg4(3)+z4*10+10;
surf(x4,y4,z4,'facecolor','g','LineStyle','none');
fill3(x4(1,:),y4(1,:),z4(1,:),'g');
fill3(x4(2,:),y4(2,:),z4(2,:),'g');
%plot3(arg4(1),arg4(2),arg4(3),'o','MarkerSize',15,'MarkerEdgeColor','w','MarkerFaceColor','k');
%plot grip
plot3([arg6(1) arg5(1)],[arg6(2) arg5(2)],[arg6(3) arg5(3)],'m','LineWidth',4);
plot3([arg7(1) arg6(1)],[arg7(2) arg6(2)],[arg7(3) arg6(3)],'m','LineWidth',4);
plot3([arg8(1) arg7(1)],[arg8(2) arg7(2)],[arg8(3) arg7(3)],'m','LineWidth',4);
%plot mini axis of each joint

grid on;
axis([-1000 1000 -1000 1000 -10 500]);
xlabel('x');
ylabel('y');
zlabel('z');

function planning(list_pos,size)
hold on;
for i=1:size-1
    plot3([list_pos(i,1) list_pos(i+1,1)],[list_pos(i,2) list_pos(i+1,2)],[list_pos(i,3) list_pos(i+1,3)],'r','LineWidth',2);
end

function struct_forward= forward_skinematic(theta,alpha,d,a) %% J1 J2 J3 J4 gr1 gr2 gr3 gr4
 DH=zeros(4,4,4);
    for i=1:4
        DH(:,:,i)=[cos(theta(i)) -cos(alpha(i))*sin(theta(i)) sin(alpha(i))*sin(theta(i)) a(i)*cos(theta(i));
                   sin(theta(i)) cos(alpha(i))*cos(theta(i)) -sin(alpha(i))*cos(theta(i)) a(i)*sin(theta(i));
                   0 sin(alpha(i)) cos(alpha(i)) d(i);
                   0 0 0 1];
    end
   J1=[0 0 383.5 1]';
   struct_forward(1,:)=J1';
   
   J2=DH(:,:,1)*[0 0 0 1]';
   struct_forward(2,:)=J2';
   
   J3=DH(:,:,1)*DH(:,:,2)*[0 0 0 1]';
   struct_forward(3,:)=J3';
   
   J4=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*[0 0 0 1]';
   struct_forward(4,:)=J4';
   
   grip1=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 -50 50 1]';
   struct_forward(5,:)=grip1';
   
   grip2=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 -50 0 1]';
   struct_forward(6,:)=grip2';
   
   grip3=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 50 0 1]';
   struct_forward(7,:)=grip3';
   
   grip4=DH(:,:,1)*DH(:,:,2)*DH(:,:,3)*DH(:,:,4)*[0 50 50 1]';
   struct_forward(8,:)=grip4';

function center =getCenter(A,B,r)
center=zeros(2,2);
syms x y
f1 =(A(1)-x)^2+(A(2)-y)^2-r^2;
f2 =(B(1)-x)^2+(B(2)-y)^2-r^2;
nghiem= solve(f1,f2,x,y);
center(1,:)=[nghiem.x(1) nghiem.y(1)]
center(2,:)=[nghiem.x(2) nghiem.y(2)]

function L = GetQmax(r,AB,condition) 
phi=acos((2*r^2-AB^2)/(2*r^2))*180/pi;
if(condition==true)
    L=2*pi*r*(phi/360);
else
    L=2*r*pi-2*r*pi*(phi/360);
end

function struct_inverse= inverse_skinematic(x,y,z,Yall,a) %%% theta1 theta2 d3 theta4
struct_inverse=zeros(6,1);
degree_to_rad = pi/180;
struct_inverse(5)=0;
struct_inverse(6)=1;
c2=(x^2+y^2-a(1)^2-a(2)^2)/(2* a(1)*a(2));
s2 =-sqrt(1-c2^2);
if(abs(c2)>1)
    struct_inverse(6)=0;
    return
end
if(atan2(s2,c2)>=0 && atan2(s2,c2)<180*pi/180 || atan2(s2,c2)<=0 && atan2(s2,c2)>-180*pi/180 )
struct_inverse(2)=atan2(s2,c2);
struct_inverse(5)=1;
end
s1=((a(1)+a(2)*c2)*y-a(2)*s2*x)/(x^2+y^2);
c1= ((a(1)+a(2)*c2)*x+a(2)*s2*y)/(x^2+ y^2);
struct_inverse(1)= atan2(s1,c1);
struct_inverse(4) = Yall*degree_to_rad - struct_inverse(1) -struct_inverse(2);
struct_inverse(3) =z- 383.5;

function get_qx_circle= get_qx_circle(qt,r)
get_qx_circle= r*cos(qt/r);

function get_qy_circle= get_qy_circle(qt,r)
get_qy_circle= r*sin(qt/r);
   