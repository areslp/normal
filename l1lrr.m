clear all;
close all;
features=load('features.txt')';
points=load('fandisk.xyz');
p_num=size(points,1);

f_dim=size(features,1);
num=size(features,2);

[Z, W, E] = l1_low_rank(features, 0.2, 10);
Z=0.5*(Z+Z');

f=fopen('n.txt','w');
for i=1:p_num
    nn=find(Z(i,:)>0);
    for j=1:length(nn)
        fprintf(f,'%d %f\n',nn(j),Z(i,nn(j)));
    end
end
fclose(f);
