% LOW RANK全局
clear all;
close all;
features=load('features.txt')';
points=load('fandisk.xyz');
p_num=size(points,1);

f_dim=size(features,1);
num=size(features,2);

% low rank参数
lambda = 0.2;
rho = 1.9;
DEBUG = 1;
[Z, E] = ladmp_lrr_fast(features, lambda, rho, DEBUG);
Z=0.5*(Z+Z');
Z(find(Z<0))=0;

for i=1:p_num
    % 将与i相似的点写入文件，作为vertex quality在meshlab中可视化
    f=fopen(['debug/' 'n' num2str(i) '.txt'],'w');
    nn=find(Z(i,:)>0);
    for j=1:length(nn)
        fprintf(f,'%d %f\n',nn(j),Z(i,nn(j)));
    end
    fclose(f);
end
