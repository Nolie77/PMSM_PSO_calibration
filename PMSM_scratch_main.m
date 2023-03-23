clear 
close all
clc

Pole_Pairs = 6;
Sim_Time   = 6;

Position_Sensor_Offset =  6 * pi / 36 ;

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

%% init
% V_an
% Zero crossing times, currently consider there is no white noise
V_an_Crosses = 0;  
V_an_Crosses_seq = zeros(1,floor(samples_num/20));

% Zero neighbours
V_an_Cnt_Zero_Crossing  = 0;
V_an_Time_Zero_Crossing = zeros(1,floor(samples_num/20));
V_an_Seq_Zero_Crossing  = zeros(1,floor(samples_num/20));
V_an_crossing_theta_m   = zeros(1,floor(samples_num/200));


% V_bn
V_bn_Crosses = 0;  
V_bn_Crosses_seq = zeros(1,floor(samples_num/20));

% Zero neighbours
V_bn_Cnt_Zero_Crossing  = 0;
V_bn_Time_Zero_Crossing = zeros(1,floor(samples_num/20));
V_bn_Seq_Zero_Crossing  = zeros(1,floor(samples_num/20));
V_bn_crossing_theta_m   = zeros(1,floor(samples_num/200));

% V_cn
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

%% Back EMF zero-crossing estimator
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

        if  abs(Simulation_Data.speed_mech.Data(i)) > 1 && abs(Simulation_Data.speed_mech.Data(i-1)) > 1 && ...
               i > ceil(0.2*samples_num) 
               
            PSO_est_cnt = PSO_est_cnt + 1;

            if Simulation_Data.V_an.Data(i) < 0
            PSO_est_o(PSO_est_cnt) =  pi / 3 - mod(V_an_crossing_theta_m(V_an_Crosses) , pi/3);
            elseif Simulation_Data.V_an.Data(i) > 0 
            PSO_est_o(PSO_est_cnt) =  pi / 3 - mod(V_an_crossing_theta_m(V_an_Crosses) + pi/6 ,pi/3);
            end

            if PSO_est_cnt == 1
            PSO_est(PSO_est_cnt) = PSO_est_o(1);
            else 
            PSO_est(PSO_est_cnt) = PSO_est(PSO_est_cnt-1) * (PSO_est_cnt-1)/PSO_est_cnt...
                + PSO_est_o(PSO_est_cnt)/PSO_est_cnt;
            end
        end   
    end

%     %% Find zero crossing point at v_bn
%     if  abs(Simulation_Data.V_bn.Data(i)) < 1e-3
%         V_bn_Cnt_Zero_Crossing = V_bn_Cnt_Zero_Crossing + 1;
%         V_bn_Time_Zero_Crossing(V_bn_Cnt_Zero_Crossing)  = Simulation_Data.V_bn.Time(i);
%         V_bn_Seq_Zero_Crossing (V_bn_Cnt_Zero_Crossing)  = i;
%     end
% 
%     if  Simulation_Data.V_bn.Data(i) * Simulation_Data.V_bn.Data(i-1) < 0 
%         V_bn_Crosses = V_bn_Crosses + 1;
%         V_bn_Crosses_seq(V_bn_Crosses) = i;
%         V_bn_crossing_theta_m(V_bn_Crosses) = Simulation_Data.position_mech.Data(i);
% 
%         if  abs(Simulation_Data.speed_mech.Data(i)) > 1 && abs(Simulation_Data.speed_mech.Data(i-1)) > 1 && ...
%                i > ceil(0.2*samples_num) 
%                
%             PSO_est_cnt = PSO_est_cnt + 1;
% 
%             if Simulation_Data.V_bn.Data(i) < 0
%             PSO_est_o(PSO_est_cnt) =  pi / 3 - mod(V_bn_crossing_theta_m(V_bn_Crosses) - pi/9 , pi/3);
%             elseif Simulation_Data.V_bn.Data(i) > 0 
%             PSO_est_o(PSO_est_cnt) =  pi / 3 - mod(V_bn_crossing_theta_m(V_bn_Crosses) + pi/6 -pi/9 ,pi/3);
%             end
% 
%             if PSO_est_cnt == 1
%             PSO_est(PSO_est_cnt) = PSO_est_o(1);
%             else 
%             PSO_est(PSO_est_cnt) = PSO_est(PSO_est_cnt-1) * (PSO_est_cnt-1)/PSO_est_cnt...
%                 + PSO_est_o(PSO_est_cnt)/PSO_est_cnt;
%             end
%         end   
%     end
% 
%     %% Find zero crossing point at v_bn
%     if  abs(Simulation_Data.V_bn.Data(i)) < 1e-3
%         V_cn_Cnt_Zero_Crossing = V_cn_Cnt_Zero_Crossing + 1;
%         V_cn_Time_Zero_Crossing(V_cn_Cnt_Zero_Crossing)  = Simulation_Data.V_cn.Time(i);
%         V_cn_Seq_Zero_Crossing (V_cn_Cnt_Zero_Crossing)  = i;
%     end
%     
%     if  Simulation_Data.V_cn.Data(i) * Simulation_Data.V_cn.Data(i-1) < 0 
%         V_cn_Crosses = V_cn_Crosses + 1;
%         V_cn_Crosses_seq(V_cn_Crosses) = i;
%         V_cn_crossing_theta_m(V_cn_Crosses) = Simulation_Data.position_mech.Data(i);
% 
%         if  abs(Simulation_Data.speed_mech.Data(i)) > 1 && abs(Simulation_Data.speed_mech.Data(i-1)) > 1 && ...
%                i > ceil(0.2*samples_num) 
%                
%             PSO_est_cnt = PSO_est_cnt + 1;
% 
%             if Simulation_Data.V_cn.Data(i) < 0
%             PSO_est_o(PSO_est_cnt) =  pi / 3 - mod(V_cn_crossing_theta_m(V_cn_Crosses) - 2 * pi/9 , pi/3);
%             elseif Simulation_Data.V_cn.Data(i) > 0 
%             PSO_est_o(PSO_est_cnt) =  pi / 3 - mod(V_cn_crossing_theta_m(V_cn_Crosses) + pi/6 - 2 * pi/9 ,pi/3);
%             end
% 
%             if PSO_est_cnt == 1
%             PSO_est(PSO_est_cnt) = PSO_est_o(1);
%             else 
%             PSO_est(PSO_est_cnt) = PSO_est(PSO_est_cnt-1) * (PSO_est_cnt-1)/PSO_est_cnt...
%                 + PSO_est_o(PSO_est_cnt)/PSO_est_cnt;
%             end
%         end   
%     end

end

%% plot
figure(3)
scatter(1:1 :PSO_est_cnt,PSO_est(1:1:PSO_est_cnt), 'r')
hold on
scatter(1:PSO_est_cnt,PSO_est_o(1:PSO_est_cnt),  'b', '+')

yline = Position_Sensor_Offset; 
line([0, PSO_est_cnt], [yline, yline], 'Color', 'black', 'LineStyle', '--','LineWidth',2)
hold off

%% atan2 methodology
% Theta_e_cal = zeros (1, floor(0.8*samples_num));
% Atan2_value_cnt          = 0;
% PSO_est_atan2            = zeros (1, floor(0.8*samples_num));
% PSO_est_atan2_o          = zeros (1, floor(0.8*samples_num));
% PSO_est_tan_cnt         = 0;
% 
% 
% for i = 2:1:samples_num
% 
%     if abs(Simulation_Data.speed_mech.Data(i)) > 1 && ...
%                 abs(Simulation_Data.speed_mech.Data(i-1)) > 1 && i > ceil(0.2*samples_num) 
%         
%         u_ab = Simulation_Data.V_an.Data(i) - Simulation_Data.V_bn.Data(i);
%         u_bc = Simulation_Data.V_bn.Data(i) - Simulation_Data.V_cn.Data(i);
%         
%         Nume_atan2 = (-2 * u_ab -u_bc) * Simulation_Data.speed_elec.Data(i);
%         Deno_atan2 = sqrt(3) * u_bc *  Simulation_Data.speed_elec.Data(i);
%    
%         if  abs(Deno_atan2) > 1e-2 && abs(Nume_atan2) > 1e-2
% 
%              Atan2_value_cnt = Atan2_value_cnt + 1;
%             
%              Theta_e_cal(Atan2_value_cnt)   = atan2(Nume_atan2, Deno_atan2);
% 
%              if Theta_e_cal(Atan2_value_cnt) < 0
%                  Theta_e_cal(Atan2_value_cnt) = Theta_e_cal(Atan2_value_cnt) + 2 * pi;
%              end
%            
%              PSO_est_atan2_o(Atan2_value_cnt) = Theta_e_cal(Atan2_value_cnt)/Pole_Pairs - mod(Simulation_Data.position_mech.Data(i) , pi/Pole_Pairs) ;       
% %              if PSO_est_atan2_o(Atan2_value_cnt) > pi/6
% %                  PSO_est_atan2_o(Atan2_value_cnt) = PSO_est_atan2_o(Atan2_value_cnt) - pi/6;
% %              elseif PSO_est_atan2_o(Atan2_value_cnt) < 0
% %                  PSO_est_atan2_o(Atan2_value_cnt) = PSO_est_atan2_o(Atan2_value_cnt) + pi/6;
% %              end
% 
%             if  Atan2_value_cnt == 1
%                 PSO_est_atan2(1) = PSO_est_atan2_o(1);
%             else 
%                 PSO_est_atan2(Atan2_value_cnt) = PSO_est_atan2(Atan2_value_cnt-1) * (Atan2_value_cnt-1)/Atan2_value_cnt...
%                     + PSO_est_atan2_o(Atan2_value_cnt)/ Atan2_value_cnt;
%             end
%          end
%     end
% end
% 
% figure(4)
% scatter(1:100 :Atan2_value_cnt,PSO_est_atan2(1:100 :Atan2_value_cnt), 'r')
% hold on
% scatter(1:Atan2_value_cnt,PSO_est_atan2_o(1:Atan2_value_cnt), 'b', '+')
% yline = Position_Sensor_Offset; 
% line([0, Atan2_value_cnt], [yline, yline], 'Color', 'black', 'LineStyle', '--','LineWidth',2)
% hold off