function [vertices,faces] = scara_stl(path)
% Read original binary STL without requiring any toolbox.
fid=fopen(path,'rb','ieee-le');
assert(fid~=-1,'scara:stl','Cannot open STL: %s',path);
cleanup=onCleanup(@()fclose(fid));
fseek(fid,80,'bof'); n=fread(fid,1,'uint32');
assert(~isempty(n) && n>0 && n<1e7,'scara:stl','Invalid binary STL.');
fseek(fid,0,'eof'); assert(ftell(fid)==84+50*n,'scara:stl','Unexpected STL length.');
fseek(fid,84,'bof'); bytes=fread(fid,[50,n],'*uint8');
data=reshape(typecast(reshape(bytes(1:48,:),[],1),'single'),12,[])';
raw=reshape(data(:,4:12)',3,[])';
[vertices,~,indices]=unique(double(raw),'rows');
faces=reshape(indices,3,[])';
end
