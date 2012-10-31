clear all;
close all;
features=load('features.txt')';
points=load('fandisk.xyz');
p_num=size(points,1);

for i=1:size(features,2)
    fprintf(1,'norm %d:%f\n',i,norm(features(:,i)));
    features(:,i)= features(:,i)/norm(features(:,i));
end

f_dim=size(features,1);
num=size(features,2);

% build kd-tree
build_params.algorithm='kdtree';
build_params.trees=5;
index=flann_build_index(points',build_params);

f=fopen('n.txt','w');
for i=1:num
    tic;
    % k-search
    k=100;
    search_params.checks=1;
    [result, dists]=flann_search(index,points(i,:)',k,search_params);
    % delete i itself
    tmp=find(result==i);
    tmp
    result(tmp)=[];
    
    D=features(:,result);
    x=features(:,i);
    lambda_max = norm(D'*x,'inf');
    lambda = 0.01*lambda_max;
    
    % ADMM without non-negative constraint
    
    %     [alpha history] = lasso(D, x, lambda, 1.0, 1.0);
    
    % yall1 non-negative ??
%     opts.tol = 5e-8;
%     opts.nonneg=1;
%     opts.rho=lambda;
%     opts.print = 0;
%     opts.nonorth=1;
%     [alpha,Out] = yall1(D, x, opts);


    % sparseLab
    alpha=SolveLasso(D,x,size(D,2),'nnlasso');
    
    data_fitting=0.5*norm(x-D*alpha)*norm(x-D*alpha);
    prior=lambda*norm(alpha,1);
    ratio=data_fitting/prior;
    nnz=length(find(alpha~=0));
    
    fd=fopen(['debug/' num2str(i) '.xyznq'],'w');
    
    for j=1:p_num
        if j==i
            fprintf(fd,'%f %f %f %f %f %f %f\n',points(j,1),points(j,2),points(j,3),0,0,0,1);
        else
            found=false;
            nnzidx=find(alpha~=0);
            for k=1:nnz
                id=nnzidx(k);
                real_id=result(id);
                if real_id==j
                    fprintf(fd,'%f %f %f %f %f %f %f\n',points(j,1),points(j,2),points(j,3),0,0,0,alpha(id));
                    found=true;
                    break;
                end
            end
            if found==false
                fprintf(fd,'%f %f %f %f %f %f %f\n',points(j,1),points(j,2),points(j,3),0,0,0,-1);
            end
        end
    end
    fclose(fd);
    
    if nnz<=5
        fprintf(1,'data_fitting: %f, prior: %f ,ratio: %f, nnz: %d\n',data_fitting, prior, ratio, nnz);
    end
    
    assert(nnz>1);
    t=toc;
    fprintf(1,'index %d point takes: %f\n',i,t);
    % break;
end
fclose(f);
