function PrintLinkName(j)
    global uLINK % Refer global variable uLINK
    if j ~= 0
        fprintf('j=%d : %s\n',j,uLINK(j).name); % Show my name
%         display('Child \n');
%         PrintLinkName(uLINK(j).child); % Show my child’s name
%         display('Sister \n');
%         PrintLinkName(uLINK(j).sister); % Show my sister’s name
    end
end
