using Pkg; Pkg.activate("VMRobotControlEnv"; shared=true)

using Printf
using Sockets
using StaticArrays
using .Threads 
using VMRobotControl

using VMRobotControl:
    DEFAULT_GRAVITY,
    DEFAULT_F_SETUP,
    DEFAULT_F_CONTROL,
    robot_ndof

#===============================================================================
# Model for communicating with ROS python client
================================================================================
There will be a ROSPY client and a Julia client. They will communicate over a 
TCP connection for commands, and a UDP connection for data. The ROS client will
listen on port 25342 for a TCP connection from Julia.

The Julia client will have 2 states: 
warmup => active

The ROS client will have 4 states: 
listening => warmup => active. 

If everything is working correctly, the ROS client will transition through these
states in order, upon receiving a "START", "WARMUP_DONE", or "STOP" message, 
through the TCP command socket.

If at any point an error occurs in Julia, a STOP message will be sent to the ROS
client, which will return to the listening state.
=#

ROSPY_LISTEN_PORT = 25342

const START = "START"
const WARMUP_DONE = "WARMUP_DONE"
const STOP = "STOP"
const STOP_ACK = "STOP_ACK"

mutable struct RobotStatePacket
    timestamp::UInt64
    sequence_number::UInt64
    state::Vector{Float64}
end

# mutable struct TorquePacket
#     timestamp::UInt64
#     sequence_number::UInt64
#     torques::Vector{Float64}
# end


mutable struct ROSPyConnectionStatus
    sequence_number::UInt64
    received_packets::UInt64
    processed_packets::UInt64
    last_received::RobotStatePacket
end

function print_connection_status_summary(connection)
    (;status_lock, status) = connection
    Base.@lock status_lock begin
        percent_received = 100 * status.received_packets / status.sequence_number
        percent_processed = 100 * status.processed_packets / status.sequence_number
        missed = status.sequence_number - status.processed_packets
        # @printf "SN: %i Received: %.3f%% Processed %.3f%% Missed: %i           \r" status.sequence_number percent_received percent_processed missed
    end
end

struct ROSPyClientConnection
    # config
    num_torques::Int
    num_states::Int
    # Sockets
    command_socket::TCPSocket
    data_socket::UDPSocket
    # State
    status::ROSPyConnectionStatus
    # Async tasks/buffers for sending/receiving data on UPD/TCP sockets
    stop::Atomic{Bool}
    recv_tcp_task::Task
    recv_tcp_lock::ReentrantLock
    tcp_buffer::IOBuffer
    recv_udp_task::Task
    status_lock::ReentrantLock
    torques::Vector{Float64}
    send_buffer::Vector{UInt8}
    function ROSPyClientConnection(
            command_socket::TCPSocket,
            num_torques::Int,
            num_states::Int
        )
        # Bind the data socket to the same address as the command socket
        @assert isopen(command_socket)
        bound_ip, bound_port = getsockname(command_socket) 
        data_socket = UDPSocket()
        bind(data_socket, bound_ip, bound_port)
        #
        status = ROSPyConnectionStatus(0, 0, 0, RobotStatePacket(0, 0, zeros(num_states)))
        # Start a task to receive commands on the tcp socket
        stop = Atomic{Bool}(false)
        recv_tcp_lock = ReentrantLock()
        tcp_buffer = PipeBuffer()
        recv_tcp_task = @async begin
            while !eof(command_socket) && !stop[]
                data = readline(command_socket)
                isempty(data) && continue
                lock(recv_tcp_lock) do
                    write(tcp_buffer, data)
                    write(tcp_buffer, '\n')
                end
                yield()
            end
            nothing
        end
        # Start a task to receive data on the data socket
        status_lock = ReentrantLock()
        recv_udp_task = @async begin
            while !stop[]
                data = recv(data_socket)
                parse_robot_state_packet!(status_lock, status, data)
                yield()
            end
        end
        # Allocate space for sending data from
        torques = zeros(num_torques)
        send_buffer = Vector{UInt8}(undef, sizeof(Int) + sizeof(Float64) + num_torques * sizeof(Float64))
        new(
            num_torques,
            num_states,
            command_socket,
            data_socket,
            status,
            stop,
            recv_tcp_task,
            recv_tcp_lock,
            tcp_buffer,
            recv_udp_task,
            status_lock,
            torques,
            send_buffer
        )
    end
end

function my_time_ns()
    UInt64(time() * 1e9)
end

function parse_robot_state_packet!(status_lock, status::ROSPyConnectionStatus, data::Vector{UInt8})
    Base.@lock status_lock begin
        expected_length = sizeof(UInt64) + sizeof(UInt64) + length(status.last_received.state) * sizeof(Float64)
        if length(data) != expected_length
            error("Invalid robot state data packet length: expected $expected_length, got $(length(data))")
        end
        b = IOBuffer(data; read=true, write=false)
        status.last_received.timestamp = ntoh(read(b, UInt64))
        status.last_received.sequence_number = ntoh(read(b, UInt64))
        for i in eachindex(status.last_received.state)
            status.last_received.state[i] = ntoh(read(b, Float64))
        end
        status.received_packets += 1
    end
    nothing
end

function _connect(rospy_ip, rospy_port, num_torques::Int, num_states::Int)
    command_socket = connect(rospy_ip, rospy_port)
    ROSPyClientConnection(command_socket, num_torques, num_states)
end

function _cleanup!(connection::ROSPyClientConnection)
    @info "Closing connection"
    write(connection.command_socket, "$STOP\n")
    # This will gracefully stop the recv tasks unless they are blocked
    connection.stop[] = true 
    sleep(0.1)
    # Closing the data socket will cause the recv_udp_task to throw an error
    close(connection.data_socket) # This causes an error 
    try
        fetch(connection.recv_udp_task)
    catch EOFError
    end
    # Closing the command socket will cause the recv_tcp_task to exit
    close(connection.command_socket)
    fetch(connection.recv_tcp_task)
    connection
end

function with_rospy_connection(f::Function, rospy_ip, rospy_port, num_torques::Int, num_states::Int)
    connection = _connect(rospy_ip, rospy_port, num_torques, num_states)
    try
        f(connection)
    finally
        _cleanup!(connection)
    end
    connection
end

function check_tcp(connection::ROSPyClientConnection)::Union{Nothing, String}
    if istaskfailed(connection.recv_tcp_task)
        fetch(connection.recv_tcp_task) # Re-throw the error
        error()
    elseif istaskdone(connection.recv_tcp_task)
        error("TCP connection/command socket closed")
    end
    yield()
    line = Base.@lock connection.recv_tcp_lock begin
        readline(connection.tcp_buffer)
    end
    if isempty(line)
        return nothing
    else
        @debug "Received command over TCP: \"$line\""
        return line
    end
end

function _get_new_packet_to_process!(connection::ROSPyClientConnection, packet::RobotStatePacket)
    @assert packet.sequence_number > connection.status.sequence_number
    connection.status.sequence_number = packet.sequence_number
    connection.status.processed_packets += 1
    deepcopy(connection.status.last_received)
end

function check_udp(connection::ROSPyClientConnection)
    if istaskfailed(connection.recv_udp_task)
        fetch(connection.recv_udp_task) # Re-throw the error
        error()
    elseif istaskdone(connection.recv_udp_task)
        error("UDP data socket closed")
    end
    # Then check if we received a packet on the data socket
    Base.@lock connection.status_lock begin
        if connection.status.last_received.sequence_number > connection.status.sequence_number
            return _get_new_packet_to_process!(connection, connection.status.last_received)
        else
            # @debug "No new UDP message to process"
            return nothing
        end
    end
end

function send_torques(connection::ROSPyClientConnection, sequence_number::UInt64)
    # Send the send buffer on the data socket
    @assert length(connection.send_buffer) == sizeof(Int) + sizeof(Float64) + connection.num_torques * sizeof(Float64)
    b = IOBuffer(connection.send_buffer; read=false, write=true)
    
    write(b, hton(sequence_number))
    write(b, hton(my_time_ns()))
    for torque in connection.torques
        write(b, hton(torque))
    end
    remote_ip, remote_port = getpeername(connection.command_socket)
    data = take!(b)
    @assert length(data) == sizeof(UInt) + sizeof(UInt) + connection.num_torques * sizeof(Float64)
    # @debug "Sending $(length(data)) bytes to $remote_ip:$remote_port"
    send(connection.data_socket, remote_ip, remote_port, data)
end


const STATE_WARMUP = 1
const STATE_ACTIVE = 2
const STATE_STOPPED = 3

function loop_warmup(connection::ROSPyClientConnection, control_func!::Function)
    @info "State: WARMUP"
    start_time = time()
    while true
        yield() # To allow for interrupts
        command = check_tcp(connection)::Union{String, Nothing}
        if isnothing(command)
            data = check_udp(connection)
            if !isnothing(data)
                i, t, dt = 0, 0.0, 0.0
                stop = control_func!(connection.torques, data.state, i, t, dt)
                stop && return STATE_STOPPED # Stop the controller gracefully
                any(isnan, connection.torques) && error("Control function returned NaN torques: $(connection.torques)")
                send_torques(connection, data.sequence_number)
                print_connection_status_summary(connection)
            end
        else
            error("Received unexpected command during warmup: \"$command\"")
        end
        if time() - start_time > 1.0
            write(connection.command_socket, "$WARMUP_DONE\n")
            return STATE_ACTIVE
        end
    end
end

function loop_active(connection::ROSPyClientConnection, control_func!::Function)
    @info "State: ACTIVE"
    i = 1
    t = 0.0
    while true
        yield() # To allow for interrupts
        command = check_tcp(connection)::Union{String, Nothing}
        if isnothing(command)
            data = check_udp(connection)
            if !isnothing(data)
                iswarmup = true
                i, dt, t = let t0 = t; t = time(); (i+1, t - t0, t) end
                stop = control_func!(connection.torques, data.state, i, t, dt)
                stop && return STATE_STOPPED # Stop the controller gracefully
                any(isnan, connection.torques) && error("Control function returned NaN torques: $(connection.torques)")
                send_torques(connection, data.sequence_number)

                print_connection_status_summary(connection)
            end
        else
            error("Received unexpected command while active: \"$command\"")
        end
    end
end

function warmup_and_activate(connection::ROSPyClientConnection, control_func!::Function; )
    if connection.stop[] != false
        error("Connection already used, create a new connection instead.")
    end

    @info "Sending $START"
    write(connection.command_socket, "$START\n")

    state = STATE_WARMUP
    while true
        if state == STATE_WARMUP
            state = loop_warmup(connection, control_func!)
        elseif state == STATE_ACTIVE
            state = loop_active(connection, control_func!)
        elseif state == STATE_STOPPED
            break
        else
            error("Invalid state: $state")
        end
    end
end

function get_initial_state(connection; retries=20, sleep_time=0.2, max_initial_state_age)
    for i = 1:retries
        Base.@lock connection.status_lock begin
            if connection.status.last_received.sequence_number > 0
                initial_state = deepcopy(connection.status.last_received)
                t_ns = my_time_ns()           
                if (t_ns - initial_state.timestamp)/1e9 > max_initial_state_age 
                    println("Initial state is too old, $((t_ns - initial_state.timestamp)/1e9)... retrying")
                    # Ensure the state is recent
                else
                    return initial_state
                end
            end
        end
        i < retries && sleep(sleep_time)
    end
    error("Failed to get initial state after $retries retries")
end

function ros_vm_controller(
        connection::ROSPyClientConnection,
        vms,
        qᵛ; 
        gravity=DEFAULT_GRAVITY,
        f_setup=DEFAULT_F_SETUP,
        f_control=DEFAULT_F_CONTROL,
        E_max=1.0,
        max_initial_state_age=1.0
    )
    # Create cache
    control_cache = new_control_cache(vms, qᵛ, gravity)
    let # Set initial joint state in cache
        state = get_initial_state(connection; max_initial_state_age)
        NDOF = robot_ndof(control_cache)
        qʳ = view(state.state, 1:NDOF)
        q̇ʳ = zeros(eltype(control_cache), NDOF)
        control_step!(control_cache, 0.0, qʳ, q̇ʳ) # Step at t=0 to set initial state
    end
    
    args = f_setup(control_cache) # Call user setup function

    # Create control callback
    control_func! = let control_cache=control_cache, args=args
        function control_func!(torques, state, i, t, dt)
            NDOF = robot_ndof(control_cache)
            @assert length(state) == 2*NDOF
            qʳ = view(state, 1:NDOF)
            q̇ʳ = view(state, NDOF+1:2*NDOF)
            # Main control step
            f_control(control_cache, t, args, (dt, i)) # Call user control function
            torques .= control_step!(control_cache, t, qʳ, q̇ʳ) # Get torques
            return false
        end
    end
    # Check that stored energy is within bounds
    (E = stored_energy(control_cache)) > E_max && error("Initial stored energy exceeds $(E_max)J, was $(E)J")
    warmup_and_activate(connection, control_func!; )
end

#=
# Enable debug logging
using Logging
debuglogger = ConsoleLogger(stderr, Logging.Debug)
global_logger(debuglogger)
=#

#=
# Test connection with python only
with_rospy_connection(Sockets.localhost, ROSPY_LISTEN_PORT+1, 7, 14) do connection
    warmup_and_activate(connection, test_control_func; )
end
=#

#=
# Test on franka
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
using FileIO, UUIDs
try
    FileIO.add_format(format"DAE", (), ".dae", [:DigitalAssetExchangeFormatIO => UUID("43182933-f65b-495a-9e05-4d939cea427d")])
catch
end
robot = parseRSON(joinpath(module_path, "RSONs/rsons/franka_panda/pandaSurgicalV2.rson"))
add_gravity_compensation!(robot, 1.1*DEFAULT_GRAVITY)

vms = compile(VirtualMechanismSystem("franka", robot))
qᵛ = Float64[]

with_rospy_connection(Sockets.localhost, ROSPY_LISTEN_PORT, 7, 14) do connection
    ros_vm_controller(connection, vms, qᵛ)
end
=#

# Test on sciurus
# Test on franka
module_path = joinpath(splitpath(splitdir(pathof(VMRobotControl))[1])[1:end-1])
robot = parseURDF(joinpath(module_path, "URDFs/sciurus17_description/urdf/sciurus17.urdf"))
add_gravity_compensation!(robot, DEFAULT_GRAVITY)

vms = VirtualMechanismSystem("sciurus", robot)
# add_gravity_compensation!(vms, -0.2*DEFAULT_GRAVITY)
add_coordinate!(vms, ReferenceCoord(Ref(SVector(.3, 0.1, 0.2))); id="l_tgt")
add_coordinate!(vms, FramePoint(".robot.l_link7", SVector(0., 0., 0.)); id="l_hand")
add_coordinate!(vms, CoordDifference("l_tgt", "l_hand"), id="l_err")
add_component!(vms, TanhSpring("l_err"; stiffness=200.0, max_force=5.0); id="l_spring")
add_component!(vms, LinearDamper(5., "l_err"); id="l_damper")

add_coordinate!(vms, ReferenceCoord(Ref(SVector(.3, 0.1, 0.2))); id="r_tgt")
add_coordinate!(vms, FramePoint(".robot.r_link7", SVector(0., 0., 0.)); id="r_hand")
add_coordinate!(vms, CoordDifference("r_tgt", "r_hand"), id="r_err")
add_component!(vms, TanhSpring("r_err"; stiffness=200.0, max_force=5.0); id="r_spring")
add_component!(vms, LinearDamper(5., "r_err"); id="r_damper")



function f_setup(cache)
    l_ref_coord_id = get_compiled_coordID(cache, "l_tgt")
    r_ref_coord_id = get_compiled_coordID(cache, "r_tgt")
    return (l_ref_coord_id, r_ref_coord_id)
end

function f_control(cache, t, setup_ret, extra)
    l_ref_coord_id, r_ref_coord_id = setup_ret
    coord = cache[l_ref_coord_id].coord_data.val[] = SVector(0.3, .1 + 0.1*sin(t), 0.2)
    coord = cache[r_ref_coord_id].coord_data.val[] = SVector(0.3 + 0.1*cos(t), -.1, 0.2 + 0.1*sin(t))
    nothing 
end

cvms = compile(vms)

qᵛ = Float64[]
with_rospy_connection(Sockets.localhost, ROSPY_LISTEN_PORT, 21, 42) do connection
    ros_vm_controller(connection, cvms, qᵛ; f_control, f_setup, E_max=2.0)
end
