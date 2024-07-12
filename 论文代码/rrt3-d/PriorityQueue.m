classdef PriorityQueue < handle
    properties
        elements % 存储元素的单元数组
    end
    
    methods
        function obj = PriorityQueue()
            obj.elements = {}; % 初始化为空单元数组
        end
        
        function insert(obj, element, priority)
            obj.elements{end + 1} = {priority, element}; % 插入元素及其优先级
        end
        
        function element = pop(obj)
            minPriority = inf;
            minIndex = -1;
            for i = 1:length(obj.elements)
                if obj.elements{i}{1} < minPriority
                    minPriority = obj.elements{i}{1};
                    minIndex = i;
                end
            end
            element = obj.elements{minIndex}{2}; % 提取对应的元素
            obj.elements(minIndex) = []; % 从队列中移除该元素
        end
        
        function isEmpty = isEmpty(obj)
            isEmpty = isempty(obj.elements); % 判断队列是否为空
        end
        
        function contains = contains(obj, element)
            contains = false; % 默认不包含
            for i = 1:length(obj.elements)
                if isequal(obj.elements{i}{2}.x, element.x) && ...
                   isequal(obj.elements{i}{2}.y, element.y) && ...
                   isequal(obj.elements{i}{2}.z, element.z)
                    contains = true; % 如果元素存在，则返回true
                    break;
                end
            end
        end
        
        function size = size(obj)
            size = length(obj.elements); % 返回队列的元素数量
        end
    end
end
