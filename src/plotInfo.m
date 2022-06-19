% pcWorld, robot, q,img, BW, obj_pos

% load data

figure(1)
subplot(2,2,1)
pcshow(pcWorld,'MarkerSize',10)
hold on
%show(m)
show(robot,q');
scatter3(posObj(1,:),posObj(2,:),posObj(3,:),'filled')
    
hold off
title('Top')
view(0,90)
axis tight

subplot(2,2,2)
imshow(img)

subplot(2,2,3)
pcshow(pcWorld,'MarkerSize',10)
hold on
%show(m)
show(robot,q');
scatter3(posObj(1,:),posObj(2,:),posObj(3,:),'filled')

hold off
title('Lateral')
view(0,0)
axis tight

subplot(2,2,4)
pcshow(pcWorld,'MarkerSize',10)
hold on
%show(m)
show(robot,q');
scatter3(posObj(1,:),posObj(2,:),posObj(3,:),'filled')

hold off
title('Isometric')
view(30,30)

axis tight


figure(2)

imshow(BW)