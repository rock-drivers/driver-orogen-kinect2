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
#    Vizkit.display task
#    Vizkit.display converter 
    Vizkit.display converter.pointcloud, :widget => Vizkit.default_loader.PointcloudVisualization
    Vizkit.exec
end
