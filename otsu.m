function level = otsu(histogramCounts, total)  
sum0 = 0;  
w0 = 0;  %w0和w1分别是一个像素可能属于前景或背景的概率
maximum = 0.0;  
total_value = sum((0:255).*histogramCounts');  
for ii=1:256  
    w0 = w0 + histogramCounts(ii);  
    if (w0 == 0)  
        continue;  %break
    end  
    w1 = total - w0;  
    if (w1 == 0)  
        break;  
    end  
    sum0 = sum0 +  (ii-1) * histogramCounts(ii);  
    m0 = sum0 / w0;  
    m1 = (total_value - sum0) / w1;  
    icv = w0 * w1 * (m0 - m1) * (m0 - m1);  % 求组间方差
    if ( icv >= maximum )  %组内方差最小化时的阈值,大津展之证明最小化组内方差（intra-class variance）与最大化组间方差（inter-class variance）是等价的
        level = ii;  
        maximum = icv;  
    end  
end 

end
