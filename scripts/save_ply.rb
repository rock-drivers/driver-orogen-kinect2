require 'orocos'
require 'vizkit'
require 'transformer/runtime' 

Orocos.initialize

widget = Vizkit.load "save.ui"
widget.show

threads = [] 


Orocos::run "kinect2_test", :output => '/dev/null' do
    task = Orocos::TaskContext.get "kinect"
    converter = Orocos::TaskContext.get "converter"
    ply = Orocos::TaskContext.get "ply"
    task.color_frame.connect_to converter.color_frame, :type => :buffer, :size => 50
    task.depth_frame.connect_to converter.frame, :type => :buffer, :size => 50

    Orocos.transformer.manager.conf.static_transform(
        Eigen::Vector3.new(0, 0, 0),
        Eigen::Quaternion.from_angle_axis(-0.5*Math::PI, Eigen::Vector3.new(0,0,1))*
        Eigen::Quaternion.from_angle_axis(-0.5*Math::PI, Eigen::Vector3.new(1,0,0)),
        "input" => "pc"
    )
    Orocos.transformer.setup(converter)
    widget.save.connect SIGNAL(:clicked) do
        threads << Thread.new(widget,ply) do |widget,ply|
            begin
                widget.save.setEnabled(false)
                widget.filename.setText("... in progess ...")
                filename = ply.saveTrigger()
                widget.filename.setText(filename)
                widget.save.setEnabled(true)
            rescue Exception => e
                STDERR.puts e
            end
        end
    end

    ply.configure
    ply.start

#    Thread.new do 
#        while true
#            STDOUT.puts threads.size
#            threads.each do |t|
#                STDOUT.puts t.status
#            end
#            sleep 1
#        end
#    end
    task.configure
    task.start
    converter.configure
    converter.start
    converter.pointcloud.connect_to ply.input, :type => :buffer, :size => 50
#    converter.pointcloud.connect_to do |pc|
#        f = File.new(find_name("output"),File::CREAT|File::TRUNC|File::RDWR, 0644)
#	    f.puts "ply"
#            f.puts "format ascii 1.0"
#            f.puts "element vertex #{pc.points.size}" 
#            f.puts "property float x"
#            f.puts "property float y"
#            f.puts "property float z"
#            f.puts "property float red"
#            f.puts "property float green"
#            f.puts "property float blue"
#            f.puts "end_header"
#
#            pc.points.each_with_index do |p,i|
#                if pc.colors.size > 0
#		    f.puts "#{p[0]} #{p[1]} #{p[2]} #{pc.colors[i].data[0]} #{pc.colors[i].data[1]} #{pc.colors[i].data[2]}"
#                else
#		    f.puts "#{p[0]} #{p[1]} #{p[2]} 1 1 1"
#                end
#            end
#            f.close
#        pc
#    end
#    Vizkit.display task
#    Vizkit.display converter 
    Vizkit.exec
end
