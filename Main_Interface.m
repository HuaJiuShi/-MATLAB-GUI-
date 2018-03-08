function varargout = Main_Interface(varargin)
% MAIN_INTERFACE MATLAB code for Main_Interface.fig
%      MAIN_INTERFACE, by itself, creates a new MAIN_INTERFACE or raises the existing
%      singleton*.
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Main_Interface_OpeningFcn, ...
                   'gui_OutputFcn',  @Main_Interface_OutputFcn, ...
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


% --- Executes just before Main_Interface is made visible.
function Main_Interface_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


% Set the colors indicating a selected/unselected tab
handles.unselectedTabColor=get(handles.tab1text,'BackgroundColor');
handles.selectedTabColor=handles.unselectedTabColor-0.1;

% Set units to normalize for easier handling
set(handles.tab1text,'Units','normalized')
set(handles.tab2text,'Units','normalized')
set(handles.tab3text,'Units','normalized')
set(handles.tab1Panel,'Units','normalized')
set(handles.tab2Panel,'Units','normalized')
set(handles.tab3Panel,'Units','normalized')

% Create tab labels (as many as you want according to following code template)

% Tab 1
pos1=get(handles.tab1text,'Position');
handles.a1=axes('Units','normalized',...
                'Box','on',...
                'XTick',[],...
                'YTick',[],...
                'Color',handles.selectedTabColor,...
                'Position',[pos1(1) pos1(2) pos1(3) pos1(4)+0.01],...
                'ButtonDownFcn','simpletab(''a1bd'',gcbo,[],guidata(gcbo))');
handles.t1=text('String','Basic',...
                'Units','normalized',...
                'Position',[(pos1(3)-pos1(1)+0.6)/2,pos1(2)/2+pos1(4)+0.1],...
                'HorizontalAlignment','left',...
                'VerticalAlignment','bottom',...
                'Margin',0.001,...
                'FontSize',12,...
                'Backgroundcolor',handles.selectedTabColor,...
                'ButtonDownFcn','simpletab(''t1bd'',gcbo,[],guidata(gcbo))');

% Tab 2
pos2=get(handles.tab2text,'Position');
pos2(1)=pos1(1)+pos1(3);
handles.a2=axes('Units','normalized',...
                'Box','on',...
                'XTick',[],...
                'YTick',[],...
                'Color',handles.unselectedTabColor,...
                'Position',[pos2(1) pos2(2) pos2(3) pos2(4)+0.01],...
                'ButtonDownFcn','simpletab(''a2bd'',gcbo,[],guidata(gcbo))');
handles.t2=text('String','Impression',...
                'Units','normalized',...
                'Position',[pos2(3)/2+0.1,pos2(2)/2+pos2(4)+0.1],...
                'HorizontalAlignment','left',...
                'VerticalAlignment','bottom',...
                'Margin',0.001,...
                'FontSize',12,...
                'Backgroundcolor',handles.unselectedTabColor,...
                'ButtonDownFcn','simpletab(''t2bd'',gcbo,[],guidata(gcbo))');
           
% Tab 3 
pos3=get(handles.tab3text,'Position');
pos3(1)=pos2(1)+pos2(3);
handles.a3=axes('Units','normalized',...
                'Box','on',...
                'XTick',[],...
                'YTick',[],...
                'Color',handles.unselectedTabColor,...
                'Position',[pos3(1) pos3(2) pos3(3) pos3(4)+0.01],...
                'ButtonDownFcn','simpletab(''a3bd'',gcbo,[],guidata(gcbo))');
handles.t3=text('String','Analysis',...
                'Units','normalized',...
                'Position',[pos3(3)/2+0.15,pos3(2)/2+pos3(4)+0.1],...
                'HorizontalAlignment','left',...
                'VerticalAlignment','bottom',...
                'Margin',0.001,...
                'FontSize',12,...
                'Backgroundcolor',handles.unselectedTabColor,...
                'ButtonDownFcn','simpletab(''t3bd'',gcbo,[],guidata(gcbo))');
            
% Manage panels (place them in the correct position and manage visibilities)
pan1pos=get(handles.tab1Panel,'Position');
set(handles.tab2Panel,'Position',pan1pos)
set(handles.tab3Panel,'Position',pan1pos)
set(handles.tab2Panel,'Visible','off')
set(handles.tab3Panel,'Visible','off')

% Update handles structure
guidata(hObject, handles);
% UIWAIT makes Main_Interface wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Main_Interface_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
varargout{1} = handles.output;


% --------------------------------------------------------------------
function file_Callback(hObject, eventdata, handles)
% hObject    handle to file (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --------------------------------------------------------------------
function openfile_Callback(hObject, eventdata, handles)
% hObject    handle to openfile (see GCBO)
[filename,pathname]=uigetfile(...
    {'*.bmp;*.jpg;*.png;*.jpeg','image files(*.bmp;*.jpg;*.png;*.jpeg)';...
        '*.*','all file(*.*)'},...
            'pick an image');
if isequal(filename,0)||isequal(pathname,0)
    return;
end

axes(handles.axes1);
fpath=strcat(pathname,filename);
imgsrc=imread(fpath);
setappdata(handles.axes1,'imgsrc',imgsrc);
imshow(imgsrc);

axes(handles.axes2);
imgsize=size(imgsrc);
if numel(imgsize)>2
    grayPic=rgb2gray(imgsrc);%实现图像矩阵的归一化操作
else
    grayPic=imgsrc;
end
setappdata(handles.axes1,'grayPic',grayPic);
setappdata(handles.axes1,'ProcessPic',grayPic);
imshow(grayPic);
imwrite(grayPic,['E:\0_Working\MATLAB\Picture_Processing_Design\GUI\','grayPic','.jpg']);

axes(handles.axes7);
ProcessPic=getappdata(handles.axes1,'ProcessPic');
setappdata(handles.axes1,'ProcessPic1',ProcessPic);
imshow(ProcessPic);

setappdata(handles.axes1,'img_next',ProcessPic);
setappdata(handles.axes1,'img_last',ProcessPic);

flag0=0;
setappdata(handles.axes1,'flag0',flag0);

set(handles.openfile,'Enable','on');

% --------------------------------------------------------------------
function savefile_Callback(hObject, eventdata, handles)
% hObject    handle to savefile (see GCBO)
[filename,pathname]=uiputfile(...
    {'*.bmp','BMP files';'*.jpg','JPG files';'*.jpeg','JPEG files'},...
            'pick an image');
if isequal(filename,0)||isequal(pathname,0)
    return;
else
    fpath=strcat(pathname,filename);
end
w=getappdata(handles.axes1,'ProcessPic');
imwrite(w,fpath);

% --- Executes on button press in pushbutton_Reset.
function pushbutton_Reset_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_Reset (see GCBO)
axes(handles.axes7);
ProcessPic=getappdata(handles.axes1,'grayPic');
setappdata(handles.axes1,'ProcessPic',ProcessPic);
setappdata(handles.axes1,'ProcessPic1',ProcessPic);

setappdata(handles.axes1,'img_next',ProcessPic);
setappdata(handles.axes1,'img_last',ProcessPic);

flag0=0;
setappdata(handles.axes1,'flag0',flag0);

imshow(ProcessPic);
set(handles.savefile,'Enable','on');

% --- Executes on button press in boundary_detection.
function boundary_detection_Callback(hObject, eventdata, handles)
% hObject    handle to boundary_detection (see GCBO)
axes(handles.axes7);
var=get(handles.Img_boundary_algorithm,'value');
b_t=str2num(get(handles.edit_boundary_threshold,'string'));
ProcessPic=getappdata(handles.axes1,'ProcessPic');
imgedge=ProcessPic;
switch var
    case 2
        imgedge=edge(ProcessPic,'roberts',b_t);%为保留图像的边缘一个像素
    case 3
        imgedge=edge(ProcessPic,'prewitt',b_t);%为保留图像的边缘一个像素
    case 4
        imgedge=edge(ProcessPic,'sobel',b_t);%为保留图像的边缘一个像素
    case 5
        imgedge=edge(ProcessPic,'log',b_t);%为保留图像的边缘一个像素
    case 6
        imgedge=edge(ProcessPic,'canny',b_t);%为保留图像的边缘一个像素
    otherwise
        imgedge=ProcessPic;
end

imshow(imgedge);
setappdata(handles.axes1,'ProcessPic',imgedge);
setappdata(handles.axes1,'ProcessPic1',imgedge);
if getappdata(handles.axes1,'flag0')
    img_last=getappdata(handles.axes1,'img_last');
else
    img_last=getappdata(handles.axes1,'img_next');
end
setappdata(handles.axes1,'img_next',imgedge);
setappdata(handles.axes1,'img_last',img_last);
set(handles.savefile,'Enable','on');

function Img_boundary_algorithm_Callback(hObject, eventdata, handles)
% hObject    handle to Img_boundary_algorithm (see GCBO)

function Img_boundary_algorithm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Img_boundary_algorithm (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit_boundary_threshold_Callback(hObject, eventdata, handles)
% hObject    handle to edit_boundary_threshold (see GCBO)

function edit_boundary_threshold_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_boundary_threshold (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in image_with_noise.
function image_with_noise_Callback(hObject, eventdata, handles)
% hObject    handle to image_with_noise (see GCBO)
axes(handles.axes7);
var=get(handles.Img_noise_algorithm,'value');
n_p=str2num(get(handles.edit6_noise_parameter,'string'));
ProcessPic=getappdata(handles.axes1,'ProcessPic');
if isbw(ProcessPic)
    ProcessPic=double(ProcessPic);
end
imgnoise=ProcessPic;
switch var
    case 2
        imgnoise=imnoise(ProcessPic,'gaussian',0,n_p);%为保留图像加噪
    case 3
        imgnoise=imnoise(ProcessPic,'poisson');%为保留图像加噪
    case 4
        imgnoise=imnoise(ProcessPic,'salt & pepper',n_p);%为保留图像加噪
    case 5
        imgnoise=imnoise(ProcessPic,'speckle',n_p);%为保留图像加噪
    otherwise
        imgnoise=ProcessPic;
end

imshow(imgnoise);
setappdata(handles.axes1,'ProcessPic',imgnoise);
setappdata(handles.axes1,'ProcessPic1',imgnoise);
if getappdata(handles.axes1,'flag0')
    img_last=getappdata(handles.axes1,'img_last');
else
    img_last=getappdata(handles.axes1,'img_next');
end
setappdata(handles.axes1,'img_next',imgnoise);
setappdata(handles.axes1,'img_last',img_last);
set(handles.savefile,'Enable','on');

function Img_noise_algorithm_Callback(hObject, eventdata, handles)
% hObject    handle to Img_noise_algorithm (see GCBO)

function Img_noise_algorithm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Img_noise_algorithm (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit6_noise_parameter_Callback(hObject, eventdata, handles)
% hObject    handle to edit6_noise_parameter (see GCBO)

function edit6_noise_parameter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6_noise_parameter (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in image_denoising.
function image_denoising_Callback(hObject, eventdata, handles)
% hObject    handle to image_denoising (see GCBO)
axes(handles.axes7);
var=get(handles.Img_denoise_algorithm,'value');
f_w=str2num(get(handles.edit7_filter_window,'string'));
ProcessPic=getappdata(handles.axes1,'ProcessPic');
imgdenoise=ProcessPic;
switch var
    case 2
        imgdenoise=medfilt2(ProcessPic,[f_w f_w]);%中值滤波
    case 3
        imgdenoise=filter2(fspecial('average',f_w),ProcessPic)/255;%均值滤波
    case 4
        imgdenoise=wiener2(ProcessPic,[f_w f_w]);%二维自适应维纳滤波
    otherwise
        imgdenoise=ProcessPic;
end

imshow(imgdenoise,[]);
setappdata(handles.axes1,'ProcessPic',imgdenoise);
setappdata(handles.axes1,'ProcessPic1',imgdenoise);
if getappdata(handles.axes1,'flag0')
    img_last=getappdata(handles.axes1,'img_last');
else
    img_last=getappdata(handles.axes1,'img_next');
end
setappdata(handles.axes1,'img_next',imgdenoise);
setappdata(handles.axes1,'img_last',img_last);
set(handles.savefile,'Enable','on');

function Img_denoise_algorithm_Callback(hObject, eventdata, handles)
% hObject    handle to Img_denoise_algorithm (see GCBO)

function Img_denoise_algorithm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Img_denoise_algorithm (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit7_filter_window_Callback(hObject, eventdata, handles)
% hObject    handle to edit7_filter_window (see GCBO)

function edit7_filter_window_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7_filter_window (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in image_enhancement.
function image_enhancement_Callback(hObject, eventdata, handles)
% hObject    handle to image_enhancement (see GCBO)
axes(handles.axes7);
ProcessPic=getappdata(handles.axes1,'ProcessPic');
var=get(handles.Img_enhance_way,'value');
enhance_p=str2num(get(handles.edit5_enhance_parameter,'string'));
if isbw(ProcessPic)
    ProcessPic=double(ProcessPic);
end
imgenhance=ProcessPic;
switch var
    case 2
        imgenhance=histeq(ProcessPic,enhance_p);    %灰度直方图均衡化
    case 3
        ProcessPic=double(ProcessPic);
        H=[0 1 0,1 enhance_p,0 1 0];
        J=conv2(ProcessPic,H,'same');
        imgenhance=ProcessPic-J;    %线性锐化滤波
    case 4
        H=fspecial('sobel');
        imgenhance=filter2(H,ProcessPic);   %Sobel算子锐化处理
    case 5
        ProcessPic=double(ProcessPic);
        [IX,IY]=gradient(ProcessPic);   %求图像梯度
        gm=sqrt(IX.*IX+IY.*IY);
        J=find(gm>=enhance_p);  %确定位置
        imgenhance(J)=gm(J);    %梯度法锐化
    case 6
        f=double(ProcessPic);[r,c]=size(f);  
        F=fft2(f);G=fftshift(F);  
        d0=enhance_p; %半径范围  
        n=2;%巴特沃斯阶次  
        a=0.5;b=2.0; %高频强调滤波传递函数系数  
        mu=floor(r/2);mv=floor(c/2);  
        for u=1:r
            for v=1:c
                d=sqrt((u-mu)^2+(v-mv)^2);  
                Hlpbtw=1/(1+0.414*(d/d0)^(2*n));
                Hhpbtw=1-Hlpbtw;
                Hhfebtw=a+b*Hhpbtw; 
                Ghfebtw(u,v)=Hhfebtw*G(u,v);  
            end  
        end  
        ghfebtw=ifftshift(Ghfebtw);  
        imgenhance=uint8(real(ifft2(ghfebtw)));
    otherwise
        imgenhance=ProcessPic;
end

setappdata(handles.axes1,'ProcessPic',imgenhance);
setappdata(handles.axes1,'ProcessPic1',imgenhance);
imshow(imgenhance,[]);
if getappdata(handles.axes1,'flag0')
    img_last=getappdata(handles.axes1,'img_last');
else
    img_last=getappdata(handles.axes1,'img_next');
end
setappdata(handles.axes1,'img_next',imgenhance);
setappdata(handles.axes1,'img_last',img_last);
set(handles.savefile,'Enable','on');

function Img_enhance_way_Callback(hObject, eventdata, handles)
% hObject    handle to Img_enhance_way (see GCBO)

function Img_enhance_way_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Img_enhance_way (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit5_enhance_parameter_Callback(hObject, eventdata, handles)
% hObject    handle to edit5_enhance_parameter (see GCBO)

function edit5_enhance_parameter_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5_enhance_parameter (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton_rotate.
function pushbutton_rotate_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_rotate (see GCBO)
axes(handles.axes7);
var=get(handles.Img_rotate_way,'value');
rotate_angle=str2num(get(handles.Img_rotate_angle,'string'));
ProcessPic1=getappdata(handles.axes1,'ProcessPic1');
ProcessPic=getappdata(handles.axes1,'ProcessPic');
imgrotate=ProcessPic;
[height,width]=size(imgrotate);
switch var
    case 2
        imgrotate=imrotate(ProcessPic1,rotate_angle,'bilinear');
    case 3
        tform=maketform('affine',[-1 0 0;0 1 0;width 0 1]);
        imgrotate=imtransform(ProcessPic,tform,'nearest');%经过水平镜像变换后的图像 
        setappdata(handles.axes1,'ProcessPic1',imgrotate);
    case 4
        tform2=maketform('affine',[1 0 0;0 -1 0;0 height 1]);  
        imgrotate=imtransform(ProcessPic,tform2,'nearest');%经过竖直镜像变换后的图像  
        setappdata(handles.axes1,'ProcessPic1',imgrotate);
    otherwise
        imgrotate=ProcessPic;
end

setappdata(handles.axes1,'ProcessPic',imgrotate);
imshow(imgrotate);
if getappdata(handles.axes1,'flag0')
    img_last=getappdata(handles.axes1,'img_last');
else
    img_last=getappdata(handles.axes1,'img_next');
end
setappdata(handles.axes1,'img_next',imgrotate);
setappdata(handles.axes1,'img_last',img_last);
set(handles.savefile,'Enable','on');


function Img_rotate_angle_Callback(hObject, eventdata, handles)
% hObject    handle to Img_rotate_angle (see GCBO)

function Img_rotate_angle_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Img_rotate_angle (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Img_rotate_way_Callback(hObject, eventdata, handles)
% hObject    handle to Img_rotate_way (see GCBO)

function Img_rotate_way_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Img_rotate_way (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

% --- Executes on button press in pushbutton_scaling.
function pushbutton_scaling_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_scaling (see GCBO)
axes(handles.axes7);
var=get(handles.Img_interpolation_algorithm,'value');
scaling_times=str2num(get(handles.edit9_scaling_times,'string'));
ProcessPic=getappdata(handles.axes1,'ProcessPic');
imgscaling=ProcessPic;
switch var
    case 2
        imgscaling=imresize(ProcessPic,scaling_times,'nearest');
    case 3
        imgscaling=imresize(ProcessPic,scaling_times,'bilinear');
    case 4
        imgscaling=imresize(ProcessPic,scaling_times,'bicubic');
    otherwise
        imgscaling=ProcessPic;
end

setappdata(handles.axes1,'ProcessPic',imgscaling);
setappdata(handles.axes1,'ProcessPic1',imgscaling);
imshow(imgscaling);
if getappdata(handles.axes1,'flag0')
    img_last=getappdata(handles.axes1,'img_last');
else
    img_last=getappdata(handles.axes1,'img_next');
end
setappdata(handles.axes1,'img_next',imgscaling);
setappdata(handles.axes1,'img_last',img_last);
set(handles.savefile,'Enable','on');

function Img_interpolation_algorithm_Callback(hObject, eventdata, handles)
% hObject    handle to Img_interpolation_algorithm (see GCBO)

function Img_interpolation_algorithm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Img_interpolation_algorithm (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit9_scaling_times_Callback(hObject, eventdata, handles)
% hObject    handle to edit9_scaling_times (see GCBO)

function edit9_scaling_times_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9_scaling_times (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Img_segmentation.
function Img_segmentation_Callback(hObject, eventdata, handles)
% hObject    handle to Img_segmentation (see GCBO)
axes(handles.axes7);
var=get(handles.Img_segment_algorithm,'value');
ProcessPic=getappdata(handles.axes1,'ProcessPic');
Imgsegmentation=ProcessPic;
switch var
    case 2  %极小值阈值分割
        [h,x]=imhist(ProcessPic);
        h=smooth(h,7);
        %求出阈值T  
        df1=diff(h);%一阶差分  
        df2=diff(df1);%二阶差分  
        [m,n]=size(df2);  
        T=0;  
        for i=1:m  
            if(abs(df1(i+1))<=0.15 && df2(i)>0)  
                T=x(i+2);%确定阈值  
                break;  
            end  
        end  
        Imgsegmentation=im2bw(ProcessPic,T/255);%转为二值图像 
    case 3  %迭代最佳阈值分割
        ZMAX=max(max(ProcessPic));           %取出最大灰度值
        ZMIN=min(min(ProcessPic));           %取出最小灰度值
        TK=(ZMAX+ZMIN)/2;
        bcal=1;
        ISIZE=size(ProcessPic);              %读出图像大小,m/n
        while(bcal)
            iForeground=0;          %定义前景和背景数
            iBackground=0;
            ForegroundSum=0;        %定义前景和背景灰度总和
            BackgroundSum=0;
             for i=1:ISIZE(1)                 %循环部分求解读下%
                 for j=1:ISIZE(2) 
                     tmp=ProcessPic(i,j);
                     if(tmp>=TK)
                         iForeground=iForeground+1;
                         ForegroundSum=ForegroundSum+double(tmp);    %前景灰度值
                    else
                         iBackground=iBackground+1;
                         BackgroundSum=BackgroundSum+double(tmp);
                     end
                 end
             end
            ZO=ForegroundSum/iForeground;       %计算前景和背景的平均值
            ZB=BackgroundSum/iBackground; 
             TKTmp=uint8(ZO+ZB)/2;
             if(TKTmp==TK )
                 bcal=0;
             else
                 TK=TKTmp;
             end             %当阈值不再变化的时候，说明迭代结束
        end
        Imgsegmentation=im2bw(ProcessPic,double(TK)/255);
    case 4 %OSTU算法目的就是计算出一连通区域的阈值，然后对该区域二值化
        counts = imhist(ProcessPic);
        [m,n] = size(ProcessPic);
        level = otsu(counts, m*n);
        output = ProcessPic;
        output(output<level) = 0;
        output(output>=level) = 255;
        Imgsegmentation=output;
    otherwise 
        Imgsegmentation=ProcessPic;
end

setappdata(handles.axes1,'ProcessPic',Imgsegmentation);
setappdata(handles.axes1,'ProcessPic1',Imgsegmentation);
imshow(Imgsegmentation,[]);
if getappdata(handles.axes1,'flag0')
    img_last=getappdata(handles.axes1,'img_last');
else
    img_last=getappdata(handles.axes1,'img_next');
end
setappdata(handles.axes1,'img_next',Imgsegmentation);
setappdata(handles.axes1,'img_last',img_last);
set(handles.savefile,'Enable','on');

function Img_segment_algorithm_Callback(hObject, eventdata, handles)
% hObject    handle to Img_segment_algorithm (see GCBO)

function Img_segment_algorithm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Img_segment_algorithm (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)

% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when figure1 is resized.
function figure1_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)

% --- Executes on button press in pushbutton_last.
function pushbutton_last_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_last (see GCBO)
axes(handles.axes7);
img_last=getappdata(handles.axes1,'img_last');
flag0=1;
setappdata(handles.axes1,'flag0',flag0);
setappdata(handles.axes1,'ProcessPic',img_last);
imshow(img_last,[]);
set(handles.savefile,'Enable','on');

% --- Executes on button press in pushbutton_next.
function pushbutton_next_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_next (see GCBO)
axes(handles.axes7);
img_next=getappdata(handles.axes1,'img_next');
flag0=0;
setappdata(handles.axes1,'flag0',flag0);
setappdata(handles.axes1,'ProcessPic',img_next);
imshow(img_next,[]);
set(handles.savefile,'Enable','on');


% --- Executes during object creation, after setting all properties.
function tab1Panel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tab1Panel (see GCBO)
function tab2Panel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tab2Panel (see GCBO)
function tab3Panel_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tab3Panel (see GCBO)
function tab1text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tab1text (see GCBO)
function tab2text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tab2text (see GCBO)
function tab3text_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tab3text (see GCBO)

% Text object 1 callback (tab 1)
function t1bd(hObject,eventdata,handles)

set(hObject,'BackgroundColor',handles.selectedTabColor)
set(handles.t2,'BackgroundColor',handles.unselectedTabColor)
set(handles.t3,'BackgroundColor',handles.unselectedTabColor)
set(handles.a1,'Color',handles.selectedTabColor)
set(handles.a2,'Color',handles.unselectedTabColor)
set(handles.a3,'Color',handles.unselectedTabColor)
set(handles.tab1Panel,'Visible','on')
set(handles.tab2Panel,'Visible','off')
set(handles.tab3Panel,'Visible','off')

% Text object 2 callback (tab 2)
function t2bd(hObject,eventdata,handles)

set(hObject,'BackgroundColor',handles.selectedTabColor)
set(handles.t1,'BackgroundColor',handles.unselectedTabColor)
set(handles.t3,'BackgroundColor',handles.unselectedTabColor)
set(handles.a2,'Color',handles.selectedTabColor)
set(handles.a1,'Color',handles.unselectedTabColor)
set(handles.a3,'Color',handles.unselectedTabColor)
set(handles.tab2Panel,'Visible','on')
set(handles.tab1Panel,'Visible','off')
set(handles.tab3Panel,'Visible','off')

% Text object 3 callback (tab 3)
function t3bd(hObject,eventdata,handles)

set(hObject,'BackgroundColor',handles.selectedTabColor)
set(handles.t1,'BackgroundColor',handles.unselectedTabColor)
set(handles.t2,'BackgroundColor',handles.unselectedTabColor)
set(handles.a3,'Color',handles.selectedTabColor)
set(handles.a1,'Color',handles.unselectedTabColor)
set(handles.a2,'Color',handles.unselectedTabColor)
set(handles.tab3Panel,'Visible','on')
set(handles.tab1Panel,'Visible','off')
set(handles.tab2Panel,'Visible','off')

% Axes object 1 callback (tab 1)
function a1bd(hObject,eventdata,handles)

set(hObject,'Color',handles.selectedTabColor)
set(handles.a2,'Color',handles.unselectedTabColor)
set(handles.a3,'Color',handles.unselectedTabColor)
set(handles.t1,'BackgroundColor',handles.selectedTabColor)
set(handles.t2,'BackgroundColor',handles.unselectedTabColor)
set(handles.t3,'BackgroundColor',handles.unselectedTabColor)
set(handles.tab1Panel,'Visible','on')
set(handles.tab2Panel,'Visible','off')
set(handles.tab3Panel,'Visible','off')

% Axes object 2 callback (tab 2)
function a2bd(hObject,eventdata,handles)

set(hObject,'Color',handles.selectedTabColor)
set(handles.a1,'Color',handles.unselectedTabColor)
set(handles.a3,'Color',handles.unselectedTabColor)
set(handles.t2,'BackgroundColor',handles.selectedTabColor)
set(handles.t1,'BackgroundColor',handles.unselectedTabColor)
set(handles.t3,'BackgroundColor',handles.unselectedTabColor)
set(handles.tab2Panel,'Visible','on')
set(handles.tab1Panel,'Visible','off')
set(handles.tab3Panel,'Visible','off')

% Axes object 3 callback (tab 3)
function a3bd(hObject,eventdata,handles)

set(hObject,'Color',handles.selectedTabColor)
set(handles.a1,'Color',handles.unselectedTabColor)
set(handles.a2,'Color',handles.unselectedTabColor)
set(handles.t3,'BackgroundColor',handles.selectedTabColor)
set(handles.t1,'BackgroundColor',handles.unselectedTabColor)
set(handles.t2,'BackgroundColor',handles.unselectedTabColor)
set(handles.tab3Panel,'Visible','on')
set(handles.tab1Panel,'Visible','off')
set(handles.tab2Panel,'Visible','off')


% --- Executes on slider movement.
function slider_light_Callback(hObject, eventdata, handles)
% hObject    handle to slider_light (see GCBO)
axes(handles.axes7);
editstr=get(handles.slider_light,'value');
set(handles.edit_light,'string',num2str(editstr));
ProcessPic1=getappdata(handles.axes1,'ProcessPic1');
if isbw(ProcessPic1)
    ProcessPic1=double(ProcessPic1);
end
img_light=imadjust(ProcessPic1,[0 1-editstr],[0 1]);

imshow(img_light);
setappdata(handles.axes1,'ProcessPic',img_light);
setappdata(handles.axes1,'img_next',img_light);
setappdata(handles.axes1,'img_last',ProcessPic1);
set(handles.savefile,'Enable','on');

% --- Executes during object creation, after setting all properties.
function slider_light_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_light (see GCBO)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit_light_Callback(hObject, eventdata, handles)
% hObject    handle to edit_light (see GCBO)
axes(handles.axes7);
editstr=get(handles.edit_light,'string');
set(handles.slider_light,'value',str2num(editstr));
ProcessPic1=getappdata(handles.axes1,'ProcessPic1');
img_light=imadjust(ProcessPic1,[0 1-str2num(editstr)],[0 1]);

setappdata(handles.axes1,'ProcessPic',img_light);
imshow(img_light);
setappdata(handles.axes1,'img_next',img_light);
setappdata(handles.axes1,'img_last',ProcessPic1);
set(handles.savefile,'Enable','on');

% --- Executes during object creation, after setting all properties.
function edit_light_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_light (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_contrast_Callback(hObject, eventdata, handles)
% hObject    handle to slider_contrast (see GCBO)
axes(handles.axes7);
editstr=get(handles.slider_contrast,'value');
set(handles.edit_contrast,'string',num2str(editstr));
ProcessPic1=getappdata(handles.axes1,'ProcessPic1');
ProcessPic1=double(ProcessPic1);
editstr=editstr*100;
img_constrast=1./(1 + (125./ (ProcessPic1 + eps)) .^editstr);

setappdata(handles.axes1,'ProcessPic',img_constrast);
imshow(img_constrast);
setappdata(handles.axes1,'img_next',img_constrast);
setappdata(handles.axes1,'img_last',ProcessPic1);
set(handles.savefile,'Enable','on');


% --- Executes during object creation, after setting all properties.
function slider_contrast_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_contrast (see GCBO)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit_contrast_Callback(hObject, eventdata, handles)
% hObject    handle to edit_contrast (see GCBO)
axes(handles.axes7);
editstr=get(handles.edit_contrast,'string');
set(handles.slider_contrast,'value',str2num(editstr));
ProcessPic1=getappdata(handles.axes1,'ProcessPic1');
ProcessPic1=double(ProcessPic1);
editstr=editstr*100;
img_constrast=1./(1 + (125./ (ProcessPic1 + eps)) .^editstr);

imshow(img_constrast);
setappdata(handles.axes1,'ProcessPic',img_constrast);
setappdata(handles.axes1,'img_next',img_constrast);
setappdata(handles.axes1,'img_last',ProcessPic1);
set(handles.savefile,'Enable','on');

% --- Executes during object creation, after setting all properties.
function edit_contrast_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_contrast (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Ing_negative.
function Ing_negative_Callback(hObject, eventdata, handles)
% hObject    handle to Ing_negative (see GCBO)
axes(handles.axes7);
ProcessPic=getappdata(handles.axes1,'ProcessPic');
if isbw(ProcessPic)
    ProcessPic=double(ProcessPic);
end
img_negative=imadjust(ProcessPic,[0 1],[1 0]);

setappdata(handles.axes1,'ProcessPic',img_negative);
setappdata(handles.axes1,'ProcessPic1',img_negative);
imshow(img_negative);
if getappdata(handles.axes1,'flag0')
    img_last=getappdata(handles.axes1,'img_last');
else
    img_last=getappdata(handles.axes1,'img_next');
end
setappdata(handles.axes1,'img_next',img_negative);
setappdata(handles.axes1,'img_last',img_last);
set(handles.savefile,'Enable','on');


% --- Executes on slider movement.
function slider_compress_Callback(hObject, eventdata, handles)
% hObject    handle to slider_compress (see GCBO)

function slider_compress_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_compress (see GCBO)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit_compress_Callback(hObject, eventdata, handles)
% hObject    handle to edit_compress (see GCBO)

function edit_compress_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_compress (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Img_gray_level.
function Img_gray_level_Callback(hObject, eventdata, handles)
% hObject    handle to Img_gray_level (see GCBO)
axes(handles.axes9);
ProcessPic=getappdata(handles.axes1,'ProcessPic');
h = imhist(ProcessPic);
h1 = h(1:1:256);
horz = 1:1:256;
bar(horz, h1);
set(gca, 'xtick', 0:50:255);

% --- Executes on button press in Img_frequency.
function Img_frequency_Callback(hObject, eventdata, handles)
% hObject    handle to Img_frequency (see GCBO)
axes(handles.axes9);
ProcessPic=getappdata(handles.axes1,'ProcessPic');
f = fft2(ProcessPic);
f = fftshift(log(1+abs(f)));
imshow(log(abs(f)),[]); %显示变换后的系数分布
colormap(jet(64));
colorbar; 

% --- Executes on button press in Img_fourier.
function Img_fourier_Callback(hObject, eventdata, handles)
% hObject    handle to Img_fourier (see GCBO)
axes(handles.axes9);
ProcessPic=getappdata(handles.axes1,'ProcessPic');
f=im2double(ProcessPic);
F=fft2(f);
F2=fftshift(F);
F3=log(1+abs(F2));
imshow(F3,[]);
%imshow(log(abs(F3)),[]); %显示变换后的系数分布
colormap(jet(64));
colorbar; 


% --- Executes on button press in Img_size.
function Img_size_Callback(hObject, eventdata, handles)
% hObject    handle to Img_size (see GCBO)
axes(handles.axes7);
ProcessPic=getappdata(handles.axes1,'ProcessPic');
img_w=str2num(get(handles.edit_imgwidth,'string'));
img_h=str2num(get(handles.edit_imghight,'string'));
[h,w]=size(ProcessPic);
if get(handles.Img_size_lock,'value')
    if img_h >= img_w
        img_w=round(w*img_h/h);
        set(handles.edit_imgwidth,'string',num2str(img_w));
    else
        img_h=round(h*img_w/w);
        set(handles.edit_imghight,'string',num2str(img_h));
    end
    img_resize=imresize(ProcessPic,[img_h img_w]);
else
    img_resize=imresize(ProcessPic,[img_h img_w]);
end

imshow(img_resize);
setappdata(handles.axes1,'ProcessPic',img_resize);
if getappdata(handles.axes1,'flag0')
    img_last=getappdata(handles.axes1,'img_last');
else
    img_last=getappdata(handles.axes1,'img_next');
end
setappdata(handles.axes1,'img_next',img_resize);
setappdata(handles.axes1,'img_last',img_last);
set(handles.savefile,'Enable','on');

function edit_imgwidth_Callback(hObject, eventdata, handles)
% hObject    handle to edit_imgwidth (see GCBO)

function edit_imgwidth_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_imgwidth (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function edit_imghight_Callback(hObject, eventdata, handles)
% hObject    handle to edit_imghight (see GCBO)

function edit_imghight_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_imghight (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Img_size_lock_Callback(hObject, eventdata, handles)
% hObject    handle to Img_size_lock (see GCBO)


% --- Executes on slider movement.
function slider_threshold_Callback(hObject, eventdata, handles) %#ok<*INUSL>
% hObject    handle to slider_threshold (see GCBO)
axes(handles.axes7);
ProcessPic1=getappdata(handles.axes1,'ProcessPic1');
thresh=get(handles.slider_threshold,'value');
Img_thresh=num2str(thresh);
set(handles.edit_threshold,'string',Img_thresh);
%thresh = graythresh(Img_thresh);     %自动确定二值化阈值
Img_threshold = im2bw(ProcessPic1,thresh);       %对图像二值化

setappdata(handles.axes1,'ProcessPic',Img_threshold);
imshow(Img_threshold);
setappdata(handles.axes1,'img_next',Img_threshold);
setappdata(handles.axes1,'img_last',ProcessPic1);
set(handles.savefile,'Enable','on');


function slider_threshold_CreateFcn(hObject, eventdata, handles) %#ok<*DEFNU,*INUSD>
% hObject    handle to slider_threshold (see GCBO)
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit_threshold_Callback(hObject, eventdata, handles)
% hObject    handle to edit_threshold (see GCBO)
axes(handles.axes7);
ProcessPic1=getappdata(handles.axes1,'ProcessPic1');
thresh=get(handles.edit_threshold,'string');
set(handles.slider_threshold,'value',str2num(thresh));
Img_thresh=str2num(thresh);
%thresh = graythresh(Img_thresh);     %自动确定二值化阈值
Img_threshold = im2bw(ProcessPic1,Img_thresh);       %对图像二值化

setappdata(handles.axes1,'ProcessPic',Img_threshold);
imshow(Img_threshold);
setappdata(handles.axes1,'img_next',Img_threshold);
setappdata(handles.axes1,'img_last',ProcessPic1);
set(handles.savefile,'Enable','on');

function edit_threshold_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit_threshold (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit27_Callback(hObject, eventdata, handles)
% hObject    handle to edit27 (see GCBO)

% --- Executes during object creation, after setting all properties.
function edit27_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit27 (see GCBO)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function Img_segmentation_CreateFcn(hObject, eventdata, handles)
