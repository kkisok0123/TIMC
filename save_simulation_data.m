function Save = save_simulation_data(i, Fin, Min, Fplant, Mplant, Fd, Md, a, Us, miui, input_error, omegaid_p, qi, errors, Save)
%% 保存仿真数据到对应的保存数组
% 输入参数：
%   i: 当前时间步索引
%   Fin, Min, Fplant, Mplant, Fd, Md, a, Us, miui, input_error, omegaid_p, qi: 控制器输出变量
%   errors: 误差结构体
%   Save: 保存数组结构体（包含所有Save数组）
% 输出参数：
%   Save: 更新后的保存数组结构体

% 保存控制输入
Save.Fplant_Save(i,:) = Fplant;
Save.Mplant_Save(i,:,:) = Mplant;
Save.Fin_Save(i,:) = Fin;
Save.Min_Save(i,:) = Min;
Save.Fd_Save(i,:) = Fd;
Save.Md_Save(i,:) = Md;
Save.a_Save(i,:,:) = a;
Save.Us_Save(i, :, :) = Us;

% 计算并保存 Us_Fin_sub
Save.Us_Fin_sub_Save(i, :, :) = Us - miui;

% 保存其他数据
Save.Input_error_all(i,:,:) = input_error;
Save.omegaid_p_Save(i,:,:) = omegaid_p;
Save.qi_Save(i,:,:) = qi;

% 保存所有误差
Save.Errors_Save.e_RL(i, :) = errors.e_RL';
Save.Errors_Save.e_Ome_L(i, :) = errors.e_Ome_L';
Save.Errors_Save.e_p_L(i, :) = errors.e_p_L';
Save.Errors_Save.e_v_L(i, :) = errors.e_v_L';
Save.Errors_Save.e_qi(i, :, :) = errors.e_qi;
Save.Errors_Save.e_sOmegai(i, :, :) = errors.e_sOmegai;
Save.Errors_Save.e_Ri(i, :, :) = errors.e_Ri;
Save.Errors_Save.e_Omegai(i, :, :) = errors.e_Omegai;

end

