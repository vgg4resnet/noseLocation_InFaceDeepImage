clc
clear
RAW_PATH = '/home/zhoujie/liveness detection/zjraw/face/';
f1 = fopen([RAW_PATH,'Depth_3.raw'], 'r');
[par1,par2,par3,par4] = textread([RAW_PATH,'Depth_3.txt'],'%d%d%d%d',1);
data0 = fread(f1, 'uint16');
fclose(f1);
img1 = reshape(data0, 400, 345);
dep_img = img1';
dep_img(find(dep_img > 800))= 0;
% figure,imshow(dep_img,[420,500]); 
face = dep_img(par2 :par2 +par4,par1 :par1 + par3); 
result = face;
[m,n]=size(face);
nose = face(round(m/4):round(m*3/4),round(n/4):round(n*3/4));
% figure(1),imshow(face,[420,500]); 
% figure(2),imshow(nose,[420,500]);


[maxa,maxi]=max(face(:));
face=maxa-face;
id1=find(face==maxa); 
face(id1)=0; 
[X,Y] = meshgrid(1:n,1:m);
% figure(1),surf(X,Y,face);
zz=face(:);xx=X(:);yy=Y(:);
XI=xx';YI=yy';ZI=zz';
id2=find(ZI==0);
XI(id2)=0;
YI(id2)=0;
% figure(3),plot3(XI,YI,ZI,'b.','MarkerSize',0.5);
% [X2,Y2] = meshgrid(1:n,1:m);
% Z2 = interp2(X,Y,face,X2,Y2,'spline');%三次样条插值
% figure,surf(XI,YI,ZI);
% figure(4),plot(YI,ZI,'b.','MarkerSize',0.5);

[maxa2,maxi2]=max(nose(:));
nose=maxa2-nose;
id3=find(nose==maxa2); 
nose(id3)=0; 
[m2,n2]=size(nose);
% [X2,Y2] = meshgrid(1:n2,1:m2);
% figure(9),surf(X2,Y2,nose);
[X3,Y3] = meshgrid(1:n2,1:m2);
zz2=nose(:);xx2=X3(:);yy2=Y3(:);
x=xx2';y=yy2';z=zz2';
nose2 = [x;y;z];
% figure(5),plot3(x,y,z,'b.','MarkerSize',0.5);
% figure(6),plot(y,z,'b.','MarkerSize',0.5);
% figure(7),plot(x,z,'b.','MarkerSize',0.5);

maybe=[];
bottomx =[];
bottomy =[];
bottomz =[];
for i = -10:10
    id4=find(x==round(n2/2) +i);
    y2 =y(id4);
    z2 =z(id4);
    [maxa3,maxi3]=max(z2(:));
    maybe=[maybe,maxi3];
    bottomx = [bottomx round(n2/2)+i];
    bottomy = [bottomy y2(maxi3)];
    bottomz = [bottomz maxa3];
%     figure(8),plot(y2,z2,'b.','MarkerSize',0.5);
end
able = 0;total = 0;
for i =1:50
    num = randperm(length(maybe),1);
    line = maybe(num);
    distance = abs([-1,maybe(num)]*[maybe;ones(1,length(maybe))]);
    total=sum(distance<5); 
    if total>able           
       able=total;
       bestline=line;
    end
end
nose_row = bestline+3;

leftx = [];
rightx = [];
leftz = [];
rightz = [];
generaly = [];
for j = 0:30
    id5=find(y==nose_row-j);
    x3 =x(id5);
    z3 =z(id5);
    % [maxa4,maxi4]=max(z3(:));
    % [maxa3,maxi3]=max(z2(:));
    % maybe=[maybe,maxi3]
    able = 0;total = 0;
    for i =1:500
        num = randperm(length(x3),3);
        samplex = x3(1,num);
        samplez = z3(1,num);
        fun = polyfit(samplex,samplez,2);
        fz3=polyval(fun,x3); 
        distance = abs(fz3-z3);
        total=sum(distance<1); 
        if fun(1)>-0.05&&fun(1)<-0.03
            if total>able           
               able=total;
               bestline2=fun;
            end
        else i =i-1;
        end

    end
    fz3=polyval(bestline2,x3);
    figure(8),plot(x3,z3,'b.',x3,fz3,'r-');
    set(gca,'YLim',[0 35]);%X轴的数据显示范围
    % hold on
    distance = abs(fz3-z3);
    bestpoint = find(distance<1);
    leftpoint = bestpoint(2);
    rightpoint = bestpoint(length(bestpoint)-1);
    % plot(x3(leftpoint),z3(leftpoint),'r.',x3(rightpoint),z3(rightpoint),'r.');
    % hold off
    leftx = [leftx x3(leftpoint)];
    rightx = [rightx x3(rightpoint)];
    leftz = [leftz z3(leftpoint)];
    rightz = [rightz z3(rightpoint)];
    generaly = [generaly nose_row-j];
end

figure(5),plot3(x,y,z,'b.','MarkerSize',0.5);
hold on
plot3(leftx,generaly,leftz,'r.',rightx,generaly,rightz,'r*');

% plot3(bottomx,bottomy,bottomz,'ro');
hold off

able = 0;total = 0;
for i =1:2000
    num = randperm(length(generaly),2);
    samx2 = generaly(1,num);
    samz2 = leftx(1,num);

    k = ((samz2(2)-samz2(1))/(samx2(2)-samx2(1)));
    b = samz2(1)-samx2(1)*k;
    lline  = [k -1 b];
    distance2=abs(lline*[generaly;leftx;ones(1,length(generaly))]);
    total=sum(distance2<6); 
    if total>able           
           able=total;
           lk =k;
           lb =b;
    end
end

able = 0;total = 0;
for i =1:2000
    num = randperm(length(generaly),2);
    samx3 = generaly(1,num);
    samz3 = rightx(1,num);
    k2 = ((samz3(2)-samz3(1))/(samx3(2)-samx3(1)));
    b2 = samz3(1)-samx3(1)*k2;
    rline  = [k2 -1 b2];
    distance3=abs(rline*[generaly;rightx;ones(1,length(generaly))]);
    total=sum(distance3<6); 
    if total>able           
           able=total;
           rk =k2;
           rb =b2;
    end
end
    
% figure(9),plot(leftx,generaly,'r.',rightx,generaly,'r*');
% hold on
funly=lk*generaly+lb;
funry=rk*generaly+rb;
% plot(funly,generaly,'b-');
% plot(funry,generaly,'b-');
bottlx=lk*nose_row+lb;
bottrx=rk*nose_row+rb;
% xy=[lb rb]*inv([1,1;-lk,-rk]);
% plot(xy(1),xy(2),'bo',bottlx,nose_row,'bo',bottrx,nose_row,'bo');
% hold off

figure(1),imshow(result,[420,500]); 
hold on
% plot(round(n/4)+xy(1),round(m/4)+xy(2),'b.',round(n/4)+bottlx,round(m/4)+nose_row,'b.',round(n/4)+bottrx,round(m/4)+nose_row,'b.');
plot(round(n/4)+0.5*(bottlx+bottrx),round(m/4)+nose_row-1.5*abs(bottlx-bottrx),'b.',round(n/4)+bottlx,round(m/4)+nose_row,'b.',round(n/4)+bottrx,round(m/4)+nose_row,'b.');
hold off


