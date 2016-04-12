% plotting all real world paths made

for i = 1:length(path_mat)

    figure(i)
    set(gca,'Color',[0.8 0.8 0.8]);
    hold on
    contourf(q_x_m(30:50,70:100),q_y_m(30:50,70:100),mag(30:50,70:100),'LineColor','none');
    caxis([0,max(max(mag))]); colormap (jet); 
    c = colorbar;
    c.Label.String = 'Current Magnitude (m/s)'; 

    quiver(q_x_m(30:50,70:100),q_y_m(30:50,70:100),u(30:50,70:100),v(30:50,70:100),'LineWidth',1,'Color','k');
    
    p = path_mat(i);
    
    p = cell2mat(p);
    
    plot(p(:,1),p(:,2),'r-x')
    
    hold off
end