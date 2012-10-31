clear all;
close all;

points=load('fandisk.xyz');

features=load('features.txt')';

for i=1:size(features,2)
    % fprintf(1,'norm %d:%f\n',i,norm(features(:,i)));
    features(:,i)= features(:,i)/norm(features(:,i));
end

f_dim=size(features,1);
num=size(features,2);

for i=1:num
    tic;
    X=[features(:,1:i-1) features(:,i+1:num)];
    I=eye(f_dim);
    % B=[X I];
    B=X;
    xi=features(:,i);
    % parameter of the optimization procedure are chosen
    param.L=size(X,1); % not more than 20 non-zeros coefficients (default: min(size(D,1),size(D,2)))
    param.lambda=0; % not more than 20 non-zeros coefficients
    param.numThreads=-1; % number of processors/cores to use; the default choice is -1
    % and uses all the cores of the machine
    param.mode=2;        % penalized formulation
    param.cholesky=false;
    param.pos=true;
    
    
    %     alpha=mexLasso(xi,B,param);
    alpha=mexOMP(xi,B,param);
    t=toc;
    % fprintf(1,'lasso takes:%f\n',t);
    
    alpha=alpha(1:num-1);
    
    l=length(find(alpha>0));
    l
    if l<5
        fprintf(1,'%d has %d nonzero\n',i,l);
    end
    alpha;
    break;
end
