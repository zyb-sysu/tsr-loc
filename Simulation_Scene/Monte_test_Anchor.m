clc; clear; close all;

%% Monte Carlo VS Anchors
% Add file path

%% Simulate uniform movement and then pick points
mst = [4,3,2,1,0.5,0.25];
snr_pt = [0,5,10,15,20,25,30,35,40,45,50]; 

num = [4,7,10,13,16,19,22,25,28,31,34,37,40];

for st = 1:1:length(num)
    monte_per = zeros(10,2);
    monte_error_mid = zeros(10,50);
    for ALL = 1:1:50
        st_x = 2;
        snr = 30;

        %% Simulation of signal strength versus distance for a single anchor
        % sta = [0,0; 10,10; 20,0; 30,30; 0,30; 25,15; 10,50; 3,40; 25,50; 15,60]; % Initialize sta position

        sta_x = randi([-20 40],num(st),1);
        sta_y = randi([-10 50],num(st),1);
        sta = [sta_x, sta_y];
        N = size(sta,1); % Wi-Fi Numbers

        x = 1:st_x:20;
        y = 1/2*(x-10).^2 ;
        M = length(x); % Sampling Numbers

        r = zeros(M,N);
        for i = 1:1:N
            r(:,i) = sqrt((x-sta(i,1)).^2 + (y-sta(i,2)).^2); % Distance
        end

        %% Mid point
        % x_mid = [15, 10, 10, 20];
        % y_mid = [15, 30, -10, -5];

        x_mid = randi([-10 30],1,10);
        y_mid = randi([-10 30],1,10);

        rm = zeros(length(x_mid),N);
        for j = 1:1:length(x_mid)
            for i = 1:1:N
                rm(j,i) = sqrt((sta(i,1)-x_mid(j)).^2 + (sta(i,2)-y_mid(j)).^2); % Distance
            end
        end

        %% Add Noise
        rn = awgn(r,snr,'measured');
        rmn = awgn(rm,snr,'measured');



        %% *********************  Monte Carlo  ************************
        pt = 500;
        N = 50;
        K = 25;

        cla = length(x);
        xp = cell(1,length(sta));
        yp = cell(1,length(sta));
        rp = cell(1,length(sta));
        err = cell(1,length(sta));
        err_min = cell(1,length(sta));
        err_min10 = cell(1,length(sta));
        result = cell(1,length(sta));
        sta_pos = cell(1,length(sta));

        for i = 1:length(sta)
            rp{i} = zeros(pt,cla);
            err{i} = zeros(pt,1);
            err_min{i} = zeros(N,10);
            result{i} = zeros(K,2);
            sta_pos{i} = zeros(10,1);
        end

        for k = 1:1:K 
            for i = 1:length(sta)
                xp{i} = rand(1,pt)*100 - 50; 
                yp{i} = rand(1,pt)*100 - 50;
            end
            for j = 1:1:N
                for i = 1:1:pt
                    for o = 1:length(sta)
                        rp{o}(i,:) = sqrt((xp{o}(i)-x).^2 + (yp{o}(i)-y).^2);
                        for c = 1:cla
                            err{o}(i) = err{o}(i) + sqrt((rp{o}(i,c) - rn(c,o)).^2);
                        end
                    end
                end

                for o = 1:length(sta)
                    err_min{o}(j,:) = mink(err{o},10);
                    [xp{o},yp{o}] = sta_generate_2d(xp{o},yp{o},err{o},j,err_min{o},pt);
                    if j < N
                        err{o} = zeros(pt,1);
                    end
                end
            end

            for o = 1:length(sta)
                err_min10{o} = mink(err{o},10);
                for sp = 1:1:10
                    sta_pos{o}(sp) = find(err{o} == err_min10{o}(sp));
                end
            end

            for o = 1:length(sta)
                wt = 1./(err_min10{o});
                result{o}(k,:) = [sum(wt'.*xp{o}(sta_pos{o}))/sum(wt), sum(wt'.*yp{o}(sta_pos{o}))/sum(wt)];
            end

        end

        %% optimal position
        sta_best = cell(1,length(sta));
        for i = 1:length(sta)
            sta_best{i} = mean(result{i});
        end

        monte_sta = [];
        monte_error = zeros(6,1);
        for i = 1:1:length(sta)
            monte_sta = [monte_sta; sta_best{i}];
            monte_error(i) = norm(sta(i,:) - monte_sta(i,:));
        end


        for mid = 1:1:length(x_mid)
            xt = monte_sta(:,1);
            yt = monte_sta(:,2);

            rp = zeros(pt,length(sta));
            err = zeros(pt,1);
            err_min = zeros(N,10);
            result = zeros(K,2);
            per_pos = zeros(10,1);

            for k = 1:1:K 
                xp = rand(1,pt)*200 - 100; 
                yp = rand(1,pt)*200 - 100;
                for j = 1:1:N
                    for i = 1:1:pt
                        for o = 1:1:length(sta)
                            rp(i,o) = sqrt((xp(i)-xt(o)).^2 + (yp(i)-yt(o)).^2)';
                        end
                        err(i) = sum(sqrt((rp(i,:) - rmn(mid,:)).^2));
                    end
                    err_min(j,:) = mink(err,10); 
                    %         [xp,yp] = per_generate(xp,yp,err,j,err_min);
                    [xp,yp] = sta_generate_2d(xp,yp,err,j,err_min,pt);
                end

                err_min10 = mink(err,10);
                for sp = 1:1:10
                    per_pos(sp) = find(err == err_min10(sp));
                end
                % result(k,:) =  [mean(xp(per_pos)),mean(yp(per_pos))];
                wt = 1./(err_min10);
                result(k,:) = [sum(wt'.*xp(per_pos))/sum(wt), sum(wt'.*yp(per_pos))/sum(wt)];
            end

            %% save optimal results
            monte_per(mid,:) = mean(result);
            monte_error_mid(mid,ALL) = norm(monte_per(mid,:) - [x_mid(mid), y_mid(mid)]);
        end

    end


    %% draw figure
    % figure;
    % axis([-50 50 -50 50]);
    % scatter(sta(:,1),sta(:,2),'black','filled');
    % hold on;
    % plot(x,y,'b-','LineWidth',2);
    % hold on;
    % scatter(x_mid,y_mid,'r^','filled');
    % hold on;
    % legend('Wi-Fi anchor','track','target');
    % xlabel('x / m')
    % ylabel('y / m')

    error_snr(st) = mean(mean(monte_error_mid));
end

figure;
plot(num,error_snr, 'b-o','LineWidth',2);
xlabel('number of anchor points')
ylabel('mean error / m')
grid on
hold on
% legend('10 dB','15 dB','20 dB','25 dB')







