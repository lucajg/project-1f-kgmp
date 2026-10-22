function drawScene = curo6r_scene(ax,n,reach,axisLength,chainStyle)
%CURO6R_SCENE Create graphics once; return a function that updates their data.
% The renderer needs only [T,p,w], so it is independent of the FK convention.

hold(ax,'on'); grid(ax,'on'); axis(ax,'equal'); view(ax,35,25);
xlabel(ax,'x (m)'); ylabel(ax,'y (m)'); zlabel(ax,'z (m)');
set(ax,'FontSize',11,'GridAlpha',.10,'Box','off');
disableDefaultInteractivity(ax);
ax.Toolbar = []; % Use the standard figure toolbar for rotation, pan, and zoom.
colors = lines(n);
chain = plot3(ax,nan,nan,nan,'LineStyle',chainStyle, ...
    'Color',[.60 .64 .69],'LineWidth',1.5,'Tag','origin-chain');
guide = gobjects(n,1); arrow = gobjects(n,1);
origin = gobjects(n,1); label = gobjects(n,1);
for i = 1:n
    guide(i) = plot3(ax,nan,nan,nan,':','Color',.55+.45*colors(i,:), ...
        'LineWidth',1.2,'Tag',sprintf('axis-guide-%d',i));
    arrow(i) = quiver3(ax,0,0,0,0,0,0,0,'Color',colors(i,:), ...
        'LineWidth',1.8,'MaxHeadSize',.5,'Tag',sprintf('axis-arrow-%d',i));
    origin(i) = plot3(ax,nan,nan,nan,'o','Color',colors(i,:), ...
        'MarkerFaceColor',colors(i,:),'MarkerSize',5,'Tag',sprintf('axis-origin-%d',i));
    label(i) = text(ax,0,0,0,sprintf(' J%d',i),'Color',.75*colors(i,:), ...
        'FontWeight','bold','FontSize',11,'Clipping','on','Tag',sprintf('axis-label-%d',i));
end
endpoint = plot3(ax,nan,nan,nan,'d','Color',[.12 .15 .19], ...
    'MarkerFaceColor',[.12 .15 .19],'MarkerSize',7,'Tag','endpoint');
endpointLabel = text(ax,0,0,0,' E','FontWeight','bold','Clipping','on','Tag','endpoint-label');
drawScene = @update;

    function update(T,p,w,selected)
        points = [zeros(3,1),p,T(1:3,4)];
        set(chain,'XData',points(1,:),'YData',points(2,:),'ZData',points(3,:));
        for j = 1:n
            % The closest point to the base origin is invariant to sliding p
            % along the axis. Thus equal axis lines get equal guide segments.
            anchor = p(:,j)-w(:,j)*dot(w(:,j),p(:,j));
            ends = anchor+[-reach,reach].*w(:,j);
            active = selected == 0 || selected == j;
            set(guide(j),'XData',ends(1,:),'YData',ends(2,:),'ZData',ends(3,:), ...
                'Visible',active);
            set(arrow(j),'XData',p(1,j),'YData',p(2,j),'ZData',p(3,j), ...
                'UData',axisLength*w(1,j),'VData',axisLength*w(2,j), ...
                'WData',axisLength*w(3,j),'Visible',active);
            set(origin(j),'XData',p(1,j),'YData',p(2,j),'ZData',p(3,j),'Visible',active);
            set(label(j),'Position',p(:,j)+1.12*axisLength*w(:,j), ...
                'Visible',active);
        end
        set(endpoint,'XData',T(1,4),'YData',T(2,4),'ZData',T(3,4));
        set(endpointLabel,'Position',T(1:3,4)-[0;0;.35]);
    end
end
