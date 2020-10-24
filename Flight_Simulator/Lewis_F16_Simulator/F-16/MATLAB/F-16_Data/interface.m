function varargout = interface(varargin)
% INTERFACE MATLAB code for interface.fig
%      INTERFACE, by itself, creates a new INTERFACE or raises the existing
%      singleton*.
%
%      H = INTERFACE returns the handle to a new INTERFACE or the handle to
%      the existing singleton*.
%
%      INTERFACE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INTERFACE.M with the given input arguments.
%
%      INTERFACE('Property','Value',...) creates a new INTERFACE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before interface_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to interface_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help interface

% Last Modified by GUIDE v2.5 26-Mar-2018 21:47:52

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @interface_OpeningFcn, ...
                   'gui_OutputFcn',  @interface_OutputFcn, ...
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


% --- Executes just before interface is made visible.
function interface_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to interface (see VARARGIN)

% Choose default command line output for interface
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes interface wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = interface_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in menu_coeff.
function menu_coeff_Callback(hObject, eventdata, handles)
% hObject    handle to menu_coeff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

opcao = get(handles.menu_coeff, 'Value');
switch opcao
    case 1
        disp('Option CX selected');
        desabilita_todas_opcoes(hObject, eventdata, handles);
    case 2
        disp('Option CY selected');
        desabilita_todas_opcoes(hObject, eventdata, handles);
        set(handles.check_beta, 'Enable', 'On');
        set(handles.check_da, 'Enable', 'On');
        set(handles.check_dr, 'Enable', 'On');
    case 3
        disp('Option CZ selected');
        desabilita_todas_opcoes(hObject, eventdata, handles);
        set(handles.check_alpha, 'Enable', 'On');
        set(handles.check_beta, 'Enable', 'On');
        set(handles.check_de, 'Enable', 'On');
    case 4
        disp('Option Cl selected');
        desabilita_todas_opcoes(hObject, eventdata, handles);
    case 5
        disp('Option Cm selected');
        desabilita_todas_opcoes(hObject, eventdata, handles);
    case 6
        disp('Option Cn selected');
        desabilita_todas_opcoes(hObject, eventdata, handles);
    case 7
        disp('Option Clda selected');
        desabilita_todas_opcoes(hObject, eventdata, handles);
    case 8
        disp('Option Cldr selected');
        desabilita_todas_opcoes(hObject, eventdata, handles);
    case 9
        disp('Option Cnda selected');
        desabilita_todas_opcoes(hObject, eventdata, handles);
    case 10
        disp('Option Cndr selected');
        desabilita_todas_opcoes(hObject, eventdata, handles);
    otherwise
        disp('ERRO: menu_coeff returned an invalid option.');
end

% Hints: contents = cellstr(get(hObject,'String')) returns menu_coeff contents as cell array
%        contents{get(hObject,'Value')} returns selected item from menu_coeff


% --- Executes during object creation, after setting all properties.
function menu_coeff_CreateFcn(hObject, eventdata, handles)
% hObject    handle to menu_coeff (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in bt_plot_coeff.
function bt_plot_coeff_Callback(hObject, eventdata, handles)
global current_plot;
opcao = get(handles.menu_coeff, 'Value');
alpha = -10:5:45;
beta = -30:5:30;
de = -25:2:25;
da = -21.5:2:21.5;
dr = -30:2:30;

switch opcao
    case 1 % CX
        [X, Y, Z] = eval_fun3(@calc_CX, alpha, de, '~');
        colormap(jet);
        current_plot = surf(handles.grafico, X, Y, Z);
        axis square;
        xlabel('\alpha (deg)');
        ylabel('\delta_e (deg)');
        zlabel('C_X');
        title('C_X as a function of \alpha and \delta_e');
        xlim(handles.grafico, [alpha(1) alpha(end)]);
        ylim(handles.grafico, [de(1) de(end)]);     
        
    case 2 % CY
        beta_check = get(handles.check_beta, 'Value');
        da_check = get(handles.check_da, 'Value');
        dr_check = get(handles.check_dr, 'Value');
        if (beta_check + da_check + dr_check ~=1)
            disp('ERRO: Only one option selected.')
        else
            if (beta_check == 1)
                beta_fixo = str2double(get(handles.txt_beta, 'String'));
                [X, Y, Z] = eval_fun3(@calc_CY, beta_fixo, da, dr);
                colormap(jet);
                current_plot = surf(handles.grafico, X, Y, Z);
                axis square;
                xlabel('\delta_a (deg)');
                ylabel('\delta_r (deg)');
                zlabel('C_Y');
                title(['C_Y as a function of \delta_a and \delta_r with \beta = ' num2str(beta_fixo) 'º']);
                xlim(handles.grafico, [da(1) da(end)]);
                ylim(handles.grafico, [dr(1) dr(end)]);    
            elseif (da_check == 1)
                da_fixo = str2double(get(handles.txt_da, 'String'));
                [X, Y, Z] = eval_fun3(@calc_CY, beta, da_fixo, dr);
                colormap(jet);
                current_plot = surf(handles.grafico, X, Y, Z);
                axis square;
                xlabel('\beta (deg)');
                ylabel('\delta_r (deg)');
                zlabel('C_Y');
                title(['C_Y as a function of \beta and \delta_r with \delta_a = ' num2str(da_fixo) 'º']);
                xlim(handles.grafico, [beta(1) beta(end)]);
                ylim(handles.grafico, [dr(1) dr(end)]);
            elseif (dr_check == 1)
                dr_fixo = str2double(get(handles.txt_dr, 'String'));
                [X, Y, Z] = eval_fun3(@calc_CY, beta, da, dr_fixo);
                colormap(jet);
                current_plot = surf(handles.grafico, X, Y, Z);
                axis square;
                xlabel('\beta (deg)');
                ylabel('\delta_a (deg)');
                zlabel('C_Y');
                title(['C_Y as a function of \beta and \delta_a with \delta_r = ' num2str(dr_fixo) 'º']);
                xlim(handles.grafico, [beta(1) beta(end)]);
                ylim(handles.grafico, [da(1) da(end)]);
            end
        end
        
    case 3 % CZ
        alpha_check = get(handles.check_alpha, 'Value');
        beta_check = get(handles.check_beta, 'Value');
        de_check = get(handles.check_de, 'Value');
        if (alpha_check + beta_check + de_check ~=1)
            disp('ERRO: Only one option selected.')
        else
            if (alpha_check == 1)
                alpha_fixo = str2double(get(handles.txt_alpha, 'String'));
                [X, Y, Z] = eval_fun3(@calc_CZ, alpha_fixo, beta, de);
                colormap(jet);
                current_plot = surf(handles.grafico, X, Y, Z);
                axis square;
                xlabel('\beta (deg)');
                ylabel('\delta_e (deg)');
                zlabel('C_Z');
                title(['C_Z as a function of \beta and \delta_e with \alpha = ' num2str(alpha_fixo) 'º']);
                xlim(handles.grafico, [beta(1) beta(end)]);
                ylim(handles.grafico, [de(1) de(end)]);
            elseif (beta_check == 1)
                beta_fixo = str2double(get(handles.txt_beta, 'String'));
                [X, Y, Z] = eval_fun3(@calc_CZ, alpha, beta_fixo, de);
                colormap(jet);
                current_plot = surf(handles.grafico, X, Y, Z);
                axis square;
                xlabel('\alpha (deg)');
                ylabel('\delta_e (deg)');
                zlabel('C_Z');
                title(['C_Z as a function of \alpha and \delta_e with \beta = ' num2str(beta_fixo) 'º']);
                xlim(handles.grafico, [alpha(1) alpha(end)]);
                ylim(handles.grafico, [de(1) de(end)]);
            elseif (de_check == 1)
                de_fixo = str2double(get(handles.txt_de, 'String'));
                [X, Y, Z] = eval_fun3(@calc_CZ, alpha, beta, de_fixo);
                colormap(jet);
                current_plot = surf(handles.grafico, X, Y, Z);
                axis square;
                xlabel('\alpha (deg)');
                ylabel('\beta (deg)');
                zlabel('C_Z');
                title(['C_Z as a function of \alpha and \beta with \delta_e = ' num2str(de_fixo) 'º']);
                xlim(handles.grafico, [alpha(1) alpha(end)]);
                ylim(handles.grafico, [beta(1) beta(end)]);
            end
        end       
        
    case 4 % Cl
        [X, Y, Z] = eval_fun3(@calc_Cl, alpha, beta, '~');
        colormap(jet);
        current_plot = surf(handles.grafico, X, Y, Z);
        axis square;
        xlabel('\alpha (deg)');
        ylabel('\beta (deg)');
        zlabel('C_l');
        title('C_l as a function of \alpha and \beta');
        xlim(handles.grafico, [alpha(1) alpha(end)]);
        ylim(handles.grafico, [beta(1) beta(end)]); 
        
    case 5 % Cm
        [X, Y, Z] = eval_fun3(@calc_Cm, alpha, de, '~');
        colormap(jet);
        current_plot = surf(handles.grafico, X, Y, Z);
        axis square;
        xlabel('\alpha (deg)');
        ylabel('\delta_e (deg)');
        zlabel('C_m');
        title('C_m as a function of \alpha and \delta_e');
        xlim(handles.grafico, [alpha(1) alpha(end)]);
        ylim(handles.grafico, [de(1) de(end)]); 
        
    case 6 % Cn
        [X, Y, Z] = eval_fun3(@calc_Cn, alpha, beta, '~');
        colormap(jet);
        current_plot = surf(handles.grafico, X, Y, Z);
        axis square;
        xlabel('\alpha (deg)');
        ylabel('\beta (deg)');
        zlabel('C_n');
        title('C_n as a function of \alpha and \beta');
        xlim(handles.grafico, [alpha(1) alpha(end)]);
        ylim(handles.grafico, [beta(1) beta(end)]); 
        
    case 7 % Clda
        [X, Y, Z] = eval_fun3(@calc_dClda, alpha, beta, '~');
        colormap(jet);
        current_plot = surf(handles.grafico, X, Y, Z);
        axis square;
        xlabel('\alpha (deg)');
        ylabel('\beta (deg)');
        zlabel('C_{l \delta a}');
        title('C_{l \delta a} as a function of \alpha and \beta');
        xlim(handles.grafico, [alpha(1) alpha(end)]);
        ylim(handles.grafico, [beta(1) beta(end)]); 
        
    case 8 % Cldr
        [X, Y, Z] = eval_fun3(@calc_dCldr, alpha, beta, '~');
        colormap(jet);
        current_plot = surf(handles.grafico, X, Y, Z);
        axis square;
        xlabel('\alpha (deg)');
        ylabel('\beta (deg)');
        zlabel('C_{l \delta r}');
        title('C_{l \delta r} as a function of \alpha and \beta');
        xlim(handles.grafico, [alpha(1) alpha(end)]);
        ylim(handles.grafico, [beta(1) beta(end)]);         
        
    case 9 % Cnda
        [X, Y, Z] = eval_fun3(@calc_dCnda, alpha, beta, '~');
        colormap(jet);
        current_plot = surf(handles.grafico, X, Y, Z);
        axis square;
        xlabel('\alpha (deg)');
        ylabel('\beta (deg)');
        zlabel('C_{n \delta a}');
        title('C_{n \delta a} as a function of \alpha and \beta');
        xlim(handles.grafico, [alpha(1) alpha(end)]);
        ylim(handles.grafico, [beta(1) beta(end)]);         
        
    case 10 % Cndr
        [X, Y, Z] = eval_fun3(@calc_dCndr, alpha, beta, '~');
        colormap(jet);
        current_plot = surf(handles.grafico, X, Y, Z);
        axis square;
        xlabel('\alpha (deg)');
        ylabel('\beta (deg)');
        zlabel('C_{n \delta r}');
        title('C_{n \delta r} as a function of \alpha and \beta');
        xlim(handles.grafico, [alpha(1) alpha(end)]);
        ylim(handles.grafico, [beta(1) beta(end)]);         
    otherwise
        disp('ERRO: menu_coeff returned an invalid option.');
end
    



function txt_alpha_Callback(hObject, eventdata, handles)
% hObject    handle to txt_alpha (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_alpha as text
%        str2double(get(hObject,'String')) returns contents of txt_alpha as a double


% --- Executes during object creation, after setting all properties.
function txt_alpha_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_alpha (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_beta_Callback(hObject, eventdata, handles)
% hObject    handle to txt_beta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_beta as text
%        str2double(get(hObject,'String')) returns contents of txt_beta as a double


% --- Executes during object creation, after setting all properties.
function txt_beta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_beta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_da_Callback(hObject, eventdata, handles)
% hObject    handle to txt_da (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_da as text
%        str2double(get(hObject,'String')) returns contents of txt_da as a double


% --- Executes during object creation, after setting all properties.
function txt_da_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_da (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_de_Callback(hObject, eventdata, handles)
% hObject    handle to txt_de (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_de as text
%        str2double(get(hObject,'String')) returns contents of txt_de as a double


% --- Executes during object creation, after setting all properties.
function txt_de_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_de (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_dr_Callback(hObject, eventdata, handles)
% hObject    handle to txt_dr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_dr as text
%        str2double(get(hObject,'String')) returns contents of txt_dr as a double


% --- Executes during object creation, after setting all properties.
function txt_dr_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_dr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in check_alpha.
function check_alpha_Callback(hObject, eventdata, handles)
checado = get(handles.check_alpha, 'Value');
%desabilita_todas_opcoes(hObject, eventdata, handles)
if (checado == 1)
    limpa_checkbox_textbox(hObject, eventdata, handles);
    set(handles.check_alpha, 'Value', 1);
    disp('Checkbox alpha enabled.');
    set(handles.txt_alpha, 'Enable', 'On');
else
    disp('Checkbox alpha disabled.');
    set(handles.txt_alpha, 'Enable', 'Off');
    set(handles.txt_alpha, 'String', '');
end


% --- Executes on button press in check_beta.
function check_beta_Callback(hObject, eventdata, handles)

checado = get(handles.check_beta, 'Value');
%desabilita_todas_opcoes(hObject, eventdata, handles)
if (checado == 1)
    limpa_checkbox_textbox(hObject, eventdata, handles);
    set(handles.check_beta, 'Value', 1);
    disp('Checkbox beta enabled.');
    set(handles.txt_beta, 'Enable', 'On');
else
    set(handles.txt_beta, 'Enable', 'Off');
    set(handles.txt_beta, 'String', '');
end




% --- Executes on button press in check_da.
function check_da_Callback(hObject, eventdata, handles)
checado = get(handles.check_da, 'Value');
%desabilita_todas_opcoes(hObject, eventdata, handles)
if (checado == 1)
    limpa_checkbox_textbox(hObject, eventdata, handles);
    set(handles.check_da, 'Value', 1);
    disp('Checkbox da enabled.');
    set(handles.txt_da, 'Enable', 'On');
else
    disp('Checkbox da disabled.');
    set(handles.txt_da, 'Enable', 'Off');
    set(handles.txt_da, 'String', '');
end


% --- Executes on button press in check_de.
function check_de_Callback(hObject, eventdata, handles)
checado = get(handles.check_de, 'Value');
%desabilita_todas_opcoes(hObject, eventdata, handles)
if (checado == 1)
    limpa_checkbox_textbox(hObject, eventdata, handles);
    set(handles.check_de, 'Value', 1);
    disp('Checkbox de enabled.');
    set(handles.txt_de, 'Enable', 'On');
else
    disp('Checkbox de disabled.');
    set(handles.txt_de, 'Enable', 'Off');
    set(handles.txt_de, 'String', '');
end


% --- Executes on button press in check_dr.
function check_dr_Callback(hObject, eventdata, handles)
checado = get(handles.check_dr, 'Value');
%desabilita_todas_opcoes(hObject, eventdata, handles)
if (checado == 1)
    limpa_checkbox_textbox(hObject, eventdata, handles);
    set(handles.check_dr, 'Value', 1);
    disp('Checkbox dr enabled.');
    set(handles.txt_dr, 'Enable', 'On');
else
    disp('Checkbox dr disabled.');
    set(handles.txt_dr, 'Enable', 'Off');
    set(handles.txt_dr, 'String', '');
end


function desabilita_todas_opcoes(hObject, eventdata, handles)
% Desabilita checkbox:
set(handles.check_alpha, 'Enable', 'Off');
set(handles.check_beta, 'Enable', 'Off');
set(handles.check_da, 'Enable', 'Off');
set(handles.check_de, 'Enable', 'Off');
set(handles.check_dr, 'Enable', 'Off');
% Desabilita textboxes:
set(handles.txt_alpha, 'Enable', 'Off');
set(handles.txt_beta, 'Enable', 'Off');
set(handles.txt_da, 'Enable', 'Off');
set(handles.txt_de, 'Enable', 'Off');
set(handles.txt_dr, 'Enable', 'Off');
% Limpa checkboxes:
set(handles.check_alpha, 'Value', 0);
set(handles.check_beta, 'Value', 0);
set(handles.check_da, 'Value', 0);
set(handles.check_de, 'Value', 0);
set(handles.check_dr, 'Value', 0);
% Limpa textboxes:
set(handles.txt_alpha, 'String', '');
set(handles.txt_beta, 'String', '');
set(handles.txt_da, 'String', '');
set(handles.txt_de, 'String', '');
set(handles.txt_dr, 'String', '');

function limpa_checkbox_textbox(hObject, eventdata, handles)
% Limpa checkboxes:
set(handles.check_alpha, 'Value', 0);
set(handles.check_beta, 'Value', 0);
set(handles.check_da, 'Value', 0);
set(handles.check_de, 'Value', 0);
set(handles.check_dr, 'Value', 0);
% Limpa textboxes:
set(handles.txt_alpha, 'String', '');
set(handles.txt_beta, 'String', '');
set(handles.txt_da, 'String', '');
set(handles.txt_de, 'String', '');
set(handles.txt_dr, 'String', '');
% Desabilita textboxes
set(handles.txt_alpha, 'Enable', 'off');
set(handles.txt_beta, 'Enable', 'off');
set(handles.txt_de, 'Enable', 'off');
set(handles.txt_da, 'Enable', 'off');
set(handles.txt_dr, 'Enable', 'off');


% --- Executes on button press in check_manete.
function check_manete_Callback(hObject, eventdata, handles)
checado = get(handles.check_manete, 'Value');
if (checado == 1)    
    disp('Checkbox throttle enabled.');
    set(handles.check_altitude, 'Value', 0);    
    set(handles.check_mach, 'Value', 0);    
    set(handles.txt_manete, 'Enable', 'On');    
    set(handles.txt_altitude, 'Enable', 'Off');
    set(handles.txt_mach, 'Enable', 'Off');
    set(handles.txt_altitude, 'String', '');
    set(handles.txt_mach, 'String', '');
else
    set(handles.txt_manete, 'String', '');
    set(handles.txt_manete, 'Enable', 'Off');
end

% --- Executes on button press in check_altitude.
function check_altitude_Callback(hObject, eventdata, handles)
checado = get(handles.check_altitude, 'Value');
if (checado == 1)    
    disp('Checkbox altitude enabled.');
    set(handles.check_manete, 'Value', 0);
    set(handles.check_mach, 'Value', 0);
    set(handles.txt_altitude, 'Enable', 'On');
    set(handles.txt_manete, 'Enable', 'Off');
    set(handles.txt_mach, 'Enable', 'Off');
    set(handles.txt_manete, 'String', '');
    set(handles.txt_mach, 'String', '');
else
    set(handles.txt_altitude, 'String', '');
    set(handles.txt_altitude, 'Enable', 'Off');
end



% --- Executes on button press in check_mach.
function check_mach_Callback(hObject, eventdata, handles)
checado = get(handles.check_mach, 'Value');
if (checado == 1)    
    disp('Checkbox Mach enabled.');
    set(handles.check_altitude, 'Value', 0);
    set(handles.check_manete, 'Value', 0);
    set(handles.txt_mach, 'Enable', 'On');
    set(handles.txt_altitude, 'Enable', 'Off');
    set(handles.txt_manete, 'Enable', 'Off');
    set(handles.txt_altitude, 'String', '');
    set(handles.txt_manete, 'String', '');
else
    set(handles.txt_mach, 'String', '');
    set(handles.txt_mach, 'Enable', 'Off');
end



function txt_manete_Callback(hObject, eventdata, handles)
% hObject    handle to txt_manete (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_manete as text
%        str2double(get(hObject,'String')) returns contents of txt_manete as a double


% --- Executes during object creation, after setting all properties.
function txt_manete_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_manete (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_altitude_Callback(hObject, eventdata, handles)
% hObject    handle to txt_altitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_altitude as text
%        str2double(get(hObject,'String')) returns contents of txt_altitude as a double


% --- Executes during object creation, after setting all properties.
function txt_altitude_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_altitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_mach_Callback(hObject, eventdata, handles)
% hObject    handle to txt_mach (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_mach as text
%        str2double(get(hObject,'String')) returns contents of txt_mach as a double


% --- Executes during object creation, after setting all properties.
function txt_mach_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_mach (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in bt_plotprop.
function bt_plotprop_Callback(hObject, eventdata, handles)
check_manete = get(handles.check_manete, 'Value');
check_altitude = get(handles.check_altitude, 'Value');
check_mach = get(handles.check_mach, 'Value');

mach = 0:0.1:1;
altitude = 0:5000:50000;
manete = 0:10:100;

if (check_manete + check_altitude + check_mach ~= 1)
    disp('Only one option must be selected');
else
    if (check_manete == 1)        
        manete_fixa = str2double(get(handles.txt_manete, 'String'));
        [X, Y, Z] = eval_fun3(@calc_thrust, manete_fixa, altitude, mach);
        colormap(jet);
        current_plot = surf(handles.grafico, X, Y, Z);
        axis square;
        xlabel('Altitude (ft)');
        ylabel('Mach');
        zlabel('Thrust (lbf)');
        %----- Modified on 2019-03-25:
        % title('Thrust as a function of altitude and Mach');
        title(['Thrust as a function of altitude and Mach with %Power = ', num2str(manete_fixa)]);
        %-----
        xlim(handles.grafico, [altitude(1) altitude(end)]);
        ylim(handles.grafico, [mach(1) mach(end)]); 
    elseif (check_altitude == 1)
        altitude_fixa = str2double(get(handles.txt_altitude, 'String'));
        [X, Y, Z] = eval_fun3(@calc_thrust, manete, altitude_fixa, mach);
        colormap(jet);
        current_plot = surf(handles.grafico, X, Y, Z);
        axis square;
        xlabel(sprintf('%s\n%s','Percentage of engine','      maximum power'));
        ylabel('Mach');
        zlabel('Thrust (lbf)');
        %----- Modified on 2019-03-25:
        % title('Thrust as a function of power and Mach');
        title(['Thrust as a function of power and Mach with altitude = ', num2str(altitude_fixa),' ft']);
        %-----
        xlim(handles.grafico, [manete(1) manete(end)]);
        ylim(handles.grafico, [mach(1) mach(end)]); 
    elseif (check_mach == 1)
        mach_fixo = str2double(get(handles.txt_mach, 'String'));
        [X, Y, Z] = eval_fun3(@calc_thrust, manete, altitude, mach_fixo);
        colormap(jet);
        current_plot = surf(handles.grafico, X, Y, Z);
        axis square;
        xlabel(sprintf('%s\n%s','Percentage of engine','      maximum power'));
        ylabel('Altitude (ft)');
        zlabel('Thrust (lbf)');
        %----- Modified on 2019-03-25:
        % title('Thrust as a function of power and altitude');
        title(['Thrust as a function of power and altitude with Mach = ', num2str(mach_fixo)]);
        %-----
        xlim(handles.grafico, [manete(1) manete(end)]);
        ylim(handles.grafico, [altitude(1) altitude(end)]); 
    end
end

function limpa_textboxes(hObject, eventdata, handles)
set(handles.txt_alpha, 'String', '');
set(handles.txt_alpha, 'Enable', 0);
set(handles.txt_beta, 'String', '');
set(handles.txt_beta, 'Enable', 0);
set(handles.txt_da, 'String', '');
set(handles.txt_da, 'Enable', 0);
set(handles.txt_de, 'String', '');
set(handles.txt_de, 'Enable', 0);
set(handles.txt_dr, 'String', '');
set(handles.txt_dr, 'Enable', 0);

% --------------------------------------------------------------------
function bt_save_ClickedCallback(hObject, eventdata, handles)
resposta = inputdlg('Please inform the file name:');
global current_plot;
%----- Modified on 2019-03-25:
% if (~isempty(resposta))   
%    saveas(handles.grafico, [resposta{1,1} '.png']);
% end
if (~isempty(resposta))   
   fh = isolate_axes(handles.grafico);
   set(fh,'Position',[0 4.23077 125 48.5]) 
   ax = get(fh);
   ax = ax.CurrentAxes;
   set(ax,'Position',[22.5 3.92308 110 37]) 
   print(fh,'-dpng',[resposta{1,1} '.png'],'-r300');
end
%-----

% --------------------------------------------------------------------
function bt_copiar_ClickedCallback(hObject, eventdata, handles)
%----- Modified on 2019-03-25:
% print -dmeta;
fh = isolate_axes(handles.grafico);
set(fh,'Position',[0 4.23077 125 48.5]) 
ax = get(fh);
ax = ax.CurrentAxes;  
set(ax,'Position',[7.5 6.42308 110 37])
print(fh,'-dmeta');
%-----
