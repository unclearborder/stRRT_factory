% ゴールノードまでの経路をプロットする
for jj = 1:length(path)-1
    tmp_line1(ii,jj) = plot3([node(path(jj)).x(1) node(path(jj+1)).x(1)],[node(path(jj)).x(2) node(path(jj+1)).x(2)],[node(path(jj)).x(3) node(path(jj+1)).x(3)],'r','LineWidth',7);
    %tmp_line2(ii,jj) = plot(node(path(jj)).x(1), node(path(jj)).x(2),'ko','MarkerSize',4,'MarkerFaceColor','k');
end

% 一個前のループでの経路と今のループでの経路が異なっているとき(つまり経路が更新されたとき)
% 古い経路のplotは削除する．
if ~isempty(min_path) % 経路をプロットするのが初回ではないとき    
    if ~isequal(min_path{ii-1},min_path{ii}) % 経路が更新されたとき
        delete(tmp_line1(1:ii-1,:))
    end
end
