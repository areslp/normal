% LOW RANK全局
clear all;
close all;
features=load('features.txt')';
points=load('fandisk.xyz');
p_num=size(points,1);

f_dim=size(features,1);
num=size(features,2);

% 将训练集归一化
for i=1:size(features,2)
    features(:,i)=features(:,i)/norm(features(:,i));
end

build_params.algorithm='kdtree';
build_params.trees=5;
index=flann_build_index(points',build_params);
% 取100个点的邻域
k=50;
search_params.checks=1;
for i=1:p_num
    [result, dists]=flann_search(index,points(i,:)',k,search_params);
    i_ne=find(result==i); % 当前点在Z里的索引
    features_ne=features(:,result);
    % low rank参数
    lambda = 0.2;
    rho = 1.9;
    DEBUG = 1;
    [Z, E] = ladmp_lrr_fast(features_ne, lambda, rho, DEBUG);
    % 非负矩阵分解
    Z=max(Z,0);
    % 归一化结果Z
    for j=1:length(Z)
        Z(:,j)=Z(:,j)/norm(Z(:,j));
    end
    % 将较小的值过滤掉
    % Z=wthresh(Z,'h',0.01);  
    Z=0.5*(Z+Z');

    % 将与i相似的点写入文件，作为vertex quality在meshlab中可视化
    f=fopen(['debug/' 'n' num2str(i) '.txt'],'w');
    nn=find(Z(i_ne,:)>0);
    for j=1:length(nn)
        idx_ne=nn(j);
        fprintf(f,'%d %f\n',result(idx_ne),Z(i_ne,idx_ne));
    end
    fclose(f);
    % break;
end
