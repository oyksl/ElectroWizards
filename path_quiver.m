x = path(:,1);
y = path(:,2);
thetas = path(:,3);
leng = zeros(length(x)-1,1);

for ind = 1:length(x)-1
    
    leng(ind) = sqrt((x(ind)-x(ind+1)).^2+(y(ind)-y(ind+1)).^2);
    
end

u = leng .* cos(thetas(1:length(thetas)-1)); 
v = leng .* sin(thetas(1:length(thetas)-1));
figure;
quiver(x(1:length(x)-1),y(1:length(y)-1),u,v);