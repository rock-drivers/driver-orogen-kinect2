require 'orocos'
require 'vizkit'

Orocos.initialize

Orocos::run "kinect2::Task" => "task" do
    task = Orocos::TaskContext.get "task"
    task.configure
    task.start
    Vizkit.display task
    Vizkit.exec
end
