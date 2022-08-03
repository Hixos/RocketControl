function export(name)
    path = sprintf("out/plot_%s.pdf", name);
    x0 = 100;
    y0 = 100;
    width = 1200;
    height = 700;
%     set(gcf,'position',[x0,y0,width,height]);
    set(gcf, 'Color', 'w');
    global savefig;
    if savefig
        
        export_fig(path);
    end
end