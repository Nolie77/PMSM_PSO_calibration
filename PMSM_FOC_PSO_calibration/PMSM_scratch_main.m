clear 
close all
clc

Pole_Pairs = 6;
Sim_Time   = 6;

Position_Sensor_Offset = pi / 18 ;

load('PMSM_parameters')

open("PMSM_model_draft.slx")

Simulation_Data = sim('PMSM_model_draft.slx');

%% Plot
figure(1)

subplot(3,1,1)
title('V_an');
ylabel('a phase BEMF (V)');
plot(Simulation_Data.V_an);

subplot(3,1,2)
title('V_bn');
ylabel('b phase BEMF (V)');
plot(Simulation_Data.V_bn);

subplot(3,1,3)
title('V_cn');
ylabel('c phase BEMF (V)');
plot(Simulation_Data.V_cn);

figure(2)
subplot(2,2,1)
title('Mechanical Position');
ylabel('Accumulated mehanical angle');
plot(Simulation_Data.position_mech);

subplot(2,2,2)
title('Mechanical speed');
ylabel('rad/s');
plot(Simulation_Data.speed_mech);

subplot(2,2,3)
title('Electrical Position')
ylabel('Accumulated electrical angle');
plot(Simulation_Data.position_elec);

subplot(2,2,4)
title('Electrical Speed')
ylabel('rad/s');
plot(Simulation_Data.speed_elec);


samples_num = length(Simulation_Data.tout);

%% V_an
% Zero crossing times, currently consider there is no white noise
V_an_Crosses = 0;  
V_an_Crosses_seq = zeros(1,floor(samples_num/20));

% Zero neighbours
V_an_Cnt_Zero_Crossing  = 0;
V_an_Time_Zero_Crossing = zeros(1,floor(samples_num/20));
V_an_Seq_Zero_Crossing  = zeros(1,floor(samples_num/20));
V_an_crossing_theta_m   = zeros(1,floor(samples_num/200));


%% V_bn
V_bn_Crosses = 0;  
V_bn_Crosses_seq = zeros(1,floor(samples_num/20));

% Zero neighbours
V_bn_Cnt_Zero_Crossing  = 0;
V_bn_Time_Zero_Crossing = zeros(1,floor(samples_num/20));
V_bn_Seq_Zero_Crossing  = zeros(1,floor(samples_num/20));
V_bn_crossing_theta_m   = zeros(1,floor(samples_num/200));

%% V_cn
V_cn_Crosses = 0;  
V_cn_Crosses_seq = zeros(1,floor(samples_num/20));

% Zero neighbours
V_cn_Cnt_Zero_Crossing  = 0;
V_cn_Time_Zero_Crossing = zeros(1,floor(samples_num/20));
V_cn_Seq_Zero_Crossing  = zeros(1,floor(samples_num/20));
V_cn_crossing_theta_m   = zeros(1,floor(samples_num/200));

PSO_est            = zeros (1, floor(samples_num/100));
PSO_est_o          = zeros (1, floor(samples_num/100));
PSO_est_cnt        = 0;


for i = 2:1:samples_num

    % Find zero neighbours
    if  abs(Simulation_Data.V_an.Data(i)) < 1e-3
        V_an_Cnt_Zero_Crossing = V_an_Cnt_Zero_Crossing + 1;
        V_an_Time_Zero_Crossing(V_an_Cnt_Zero_Crossing)  = Simulation_Data.V_an.Time(i);
        V_an_Seq_Zero_Crossing (V_an_Cnt_Zero_Crossing)  = i;
    end
    
    % Find zero crossing point at v_an
    if  Simulation_Data.V_an.Data(i) * Simulation_Data.V_an.Data(i-1) < 0
        V_an_Crosses = V_an_Crosses + 1;
        V_an_Crosses_seq(V_an_Crosses) = i;
        V_an_crossing_theta_m(V_an_Crosses) = Simulation_Data.position_mech.Data(i);

        if  abs(Simulation_Data.speed_mech.Data(i)) > 1 && ...
                abs(Simulation_Data.speed_mech.Data(i-1)) > 1 && i > ceil(0.2*samples_num) 

            PSO_est_cnt = PSO_est_cnt + 1;
            PSO_est_o(PSO_est_cnt) = pi/6 - mod(V_an_crossing_theta_m(V_an_Crosses),pi/6);

            if PSO_est_cnt == 1
            PSO_est(PSO_est_cnt) = PSO_est_o(1);
            else 
            PSO_est(PSO_est_cnt) = PSO_est(PSO_est_cnt-1) * (PSO_est_cnt-1)/PSO_est_cnt...
                + PSO_est_o(PSO_est_cnt)/PSO_est_cnt;
            end
        end    
    end

       %% Find zero crossing point at v_bn
    if  Simulation_Data.V_bn.Data(i) * Simulation_Data.V_bn.Data(i-1) < 0
        V_bn_Crosses = V_bn_Crosses + 1;
        V_bn_Crosses_seq(V_bn_Crosses) = i;
        V_bn_crossing_theta_m(V_bn_Crosses) = Simulation_Data.position_mech.Data(i);

        if  abs(Simulation_Data.speed_mech.Data(i)) > 1 && ...
                abs(Simulation_Data.speed_mech.Data(i-1)) > 1 && i > ceil(0.2*samples_num) 

            PSO_est_cnt = PSO_est_cnt + 1;
            PSO_est_o(PSO_est_cnt) =   pi/9 + pi/6 - mod(V_bn_crossing_theta_m(V_bn_Crosses),pi/6);
            PSO_est_o(PSO_est_cnt) = mod (PSO_est_o(PSO_est_cnt), pi/6);

            if PSO_est_cnt == 1
            PSO_est(PSO_est_cnt) = PSO_est_o(1);
            else 
            PSO_est(PSO_est_cnt) = PSO_est(PSO_est_cnt-1) * (PSO_est_cnt-1)/PSO_est_cnt...
                + PSO_est_o(PSO_est_cnt)/PSO_est_cnt;
            end
        end    
    end

%         % Find zero crossing point at v_cn
%     if  Simulation_Data.V_cn.Data(i) * Simulation_Data.V_cn.Data(i-1) < 0
%         V_cn_Crosses = V_cn_Crosses + 1;
%         V_cn_Crosses_seq(V_cn_Crosses) = i;
%         V_cn_crossing_theta_m(V_cn_Crosses) = Simulation_Data.position_mech.Data(i);
% 
%         if  abs(Simulation_Data.speed_mech.Data(i)) > 1 && ...
%                 abs(Simulation_Data.speed_mech.Data(i-1)) > 1 && i > ceil(0.2*samples_num) 
% 
%             PSO_est_cnt = PSO_est_cnt + 1;
%             PSO_est_o(PSO_est_cnt) = pi*2/9 + pi/6 - mod(V_cn_crossing_theta_m(V_cn_Crosses),pi/6);
%             PSO_est_o(PSO_est_cnt) = mod(PSO_est_o(PSO_est_cnt),pi/6);
% 
%             if PSO_est_cnt == 1
%             PSO_est(PSO_est_v_an_cnt) = PSO_est_o(1);
%             else 
%             PSO_est(PSO_est_cnt) = PSO_est(PSO_est_cnt-1) * (PSO_est_cnt-1)/PSO_est_cnt...
%                 + PSO_est_o(PSO_est_cnt)/PSO_est_cnt;
%             end
%         end    
%     end

end

figure(3)
scatter(1:PSO_est_cnt,PSO_est(1:PSO_est_cnt), 'r')
hold on
scatter(1:PSO_est_cnt,PSO_est_o(1:PSO_est_cnt),  'b')

yline = Position_Sensor_Offset; 
line([0, PSO_est_cnt], [yline, yline], 'Color', 'black', 'LineStyle', '--','LineWidth',2)
hold off


Tan_value   = zeros (1, floor(0.8*samples_num));
Theta_e_cal = zeros (1, floor(0.8*samples_num));
Tan_value_cnt          = 0;
PSO_est_tan            = zeros (1, floor(0.8*samples_num));
PSO_est_tan_o          = zeros (1, floor(0.8*samples_num));
PSO_est_tan_cnt        = 0;


for i = 2:1:samples_num

    if abs(Simulation_Data.speed_mech.Data(i)) > 1 && ...
                abs(Simulation_Data.speed_mech.Data(i-1)) > 1 && i > ceil(0.2*samples_num) 
    
        Flag_tan = sqrt(3) * Simulation_Data.V_an.Data(i) / (Simulation_Data.V_bn.Data(i) - Simulation_Data.V_cn.Data(i) )  ;
   
        if  abs(Flag_tan) < 7

             Tan_value_cnt = Tan_value_cnt + 1;
             Tan_value(Tan_value_cnt) =  Flag_tan;
             Theta_e_cal(Tan_value_cnt)   = atan(Flag_tan);
             
             Tan_temp = Theta_e_cal(Tan_value_cnt);
             Mod_temp = mod (Simulation_Data.position_mech.Data(i)*Pole_Pairs, 2*pi);

                 if Tan_temp < 0 && Mod_temp < pi
                    PSO_est_tan_o(Tan_value_cnt) = pi + Tan_temp - Mod_temp;
    
                 elseif Tan_temp < 0 && Mod_temp > pi
                    PSO_est_tan_o(Tan_value_cnt) = 2*pi + Tan_temp - Mod_temp;
    
                 elseif Tan_temp > 0 && Mod_temp < pi
                    PSO_est_tan_o(Tan_value_cnt) = Tan_temp - Mod_temp;
    
                 elseif Tan_temp > 0 && Mod_temp > pi
                    PSO_est_tan_o(Tan_value_cnt) = pi + Tan_temp - Mod_temp;
    
                 end
            

            if  Tan_value_cnt == 1
                PSO_est_tan(1) = PSO_est_tan_o(1);
            else 
                PSO_est_tan(Tan_value_cnt) = PSO_est_tan(Tan_value_cnt-1) * (Tan_value_cnt-1)/Tan_value_cnt...
                    + PSO_est_tan_o(Tan_value_cnt)/ Tan_value_cnt;
            end
         end
    end
end

% figure(4)
% scatter(1:Tan_value_cnt,PSO_est_tan(1:Tan_value_cnt), 'r')
% hold on
% yline = Position_Sensor_Offset; 
% line([0, Tan_value_cnt], [yline, yline], 'Color', 'black', 'LineStyle', '--','LineWidth',2)
% hold off