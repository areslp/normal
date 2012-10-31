clear all;
close all;
features=load('features.txt')';

f_dim=size(features,1);
num=size(features,2);

for i=1:num
    tic;
    X=[features(:,1:i-1) features(:,i+1:num)];
    I=eye(f_dim);
    B=[X I];
    xi=features(:,i);
    
    lambda=0.5;
    d=num-1+f_dim;
    
    cvx_begin
    variable a(d,1)
    minimize( 0.5*norm(xi-B*a) + lambda*norm(a,1) )
    subject to
        for j=1:num-1+f_dim
           a(j)>=0; 
        end
    cvx_end
    
    break;
end