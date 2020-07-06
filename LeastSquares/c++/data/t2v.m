function v = t2v(t)
v = t(:,1);
v(1:2) = t(1:2,3);
v(3) = atan2(t(2,1),t(1,1));

end

