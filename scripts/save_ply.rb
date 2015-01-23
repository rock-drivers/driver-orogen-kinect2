require 'orocos'
require 'vizkit'

Orocos.initialize

def find_name(basename)
    if not File.exists?("#{basename}.ply")
        return "#{basename}.ply"
    end
    cnt = 0
    loop do
        if File.exists?("#{basename}.#{cnt}.ply")
            cnt = cnt + 1
            next
        end
        return "#{basename}.#{cnt}.ply" 
    end
end

#Orocos::run "kinect2::Task" => "task", "image_preprocessing::DepthImage2Pointcloud" => "converter", :output => '%m-%p.log' do
Orocos::run "kinect2::Task" => "task", "image_preprocessing::DepthImage2Pointcloud" => "converter", :output => '/dev/null' do
    task = Orocos::TaskContext.get "task"
    converter = Orocos::TaskContext.get "converter"
    task.color_frame.connect_to converter.color_frame, :type => :buffer, :size => 50
    task.depth_frame.connect_to converter.frame, :type => :buffer, :size => 50
    task.configure
    task.start
    converter.configure
    converter.start
    converter.pointcloud.connect_to do |pc|
        f = File.new(find_name("output"),File::CREAT|File::TRUNC|File::RDWR, 0644)
	    f.puts "ply"
            f.puts "format ascii 1.0"
            f.puts "element vertex #{pc.points.size}" 
            f.puts "property float x"
            f.puts "property float y"
            f.puts "property float z"
            f.puts "property float red"
            f.puts "property float green"
            f.puts "property float blue"
            f.puts "end_header"

            pc.points.each_with_index do |p,i|
                if pc.colors.size > 0
		    f.puts "#{p[0]} #{p[1]} #{p[2]} #{pc.colors[i].data[0]} #{pc.colors[i].data[1]} #{pc.colors[i].data[2]}"
                else
		    f.puts "#{p[0]} #{p[1]} #{p[2]} 1 1 1"
                end
            end
            f.close
        pc
    end
    Vizkit.display task
    Vizkit.display converter 
    Vizkit.exec
end
