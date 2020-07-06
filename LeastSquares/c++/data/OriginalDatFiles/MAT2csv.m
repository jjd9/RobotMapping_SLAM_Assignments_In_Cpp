clc
clear all
close all

directory = dir("*.mat");

for j = 1:length(directory)
    filename = directory(j).name
    load(filename);
    x = g.x;
    IDS = fieldnames(g.idLookup);
    idLookup = zeros(length(IDS),3);
    for i= 1:length(IDS)
        id = cell2mat(IDS(i));
        idLookup(i,1) = str2double(id);
        field = getfield(g.idLookup, id);
        idLookup(i,2) = field.offset;
        idLookup(i,3) = field.dimension;
    end

    Nedges = length(g.edges);
    type = [];
    from = zeros(Nedges,1);
    to = zeros(Nedges,1);
    measurement = zeros(Nedges,3)-999.95;
    information = zeros(Nedges,9)-999.95;
    fromIdx = zeros(Nedges,1);
    toIdx = zeros(Nedges,1);

    for i = 1:Nedges
        edge = g.edges(i);
        type(i)=edge.type;
        from(i)=edge.from;
        to(i)=edge.to;
        measurement(i,1:length(edge.measurement))=edge.measurement';
        information(i,1:length(edge.information(:)))=edge.information(:)';
        fromIdx(i)=edge.fromIdx;
        toIdx(i)=edge.toIdx;
    end

    xData = x;
    edgeData = [type(:), from(:), to(:), measurement, information, fromIdx(:), toIdx(:)];
    idLookupData = idLookup;

    
    fid = fopen(strrep(filename,".mat","_idLookup.csv"), 'wt');
    fprintf(fid, '%s'+strjoin(repelem(",%s",2))+'\n', "id", "offset", "dimension");
    fprintf(fid, '%g'+strjoin(repelem(",%g",2))+'\n', idLookupData.');   %transpose is important!
    fclose(fid);    
    
    fid = fopen(strrep(filename,".mat","_edge.csv"), 'wt');
    fprintf(fid, '%s'+strjoin(repelem(",%s",16))+'\n', 'type', 'from', 'to', ...
                'm1','m2','m3',...
                'i1','i2','i3','i4','i5','i6','i7','i8','i9',...
                'fromIdx','toIdx');
    fprintf(fid, '%g'+strjoin(repelem(",%g",16))+'\n', edgeData.');   %transpose is important!
    fclose(fid);

end