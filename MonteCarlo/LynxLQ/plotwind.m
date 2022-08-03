function plotwind(t, h, dir, mag)
for ih = 1:3
    nexttile(t)
    
    [U,V] = pol2cart(dir(ih), mag(ih));  
    compass(U,V);
    grid on
end
end