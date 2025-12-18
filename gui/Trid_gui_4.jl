using Gtk
using Gtk.ShortNames
using LibSerialPort

import Gtk: MessageType, ButtonsType


# ======== Configuration ========
const PORT = "/dev/ttyUSB0"  # Replace with your USB-serial device
const BAUD = 9600             # Baud rate

# ======== Create Window and Grid ========
win = Window("TRIDENT: USBL Simulation", 600, 750)
grid = Grid()
set_gtk_property!(grid, :row_spacing, 10)
set_gtk_property!(grid, :column_spacing, 10)
set_gtk_property!(grid, :margin, 20)

# ======== Create Status Label ========
status_label = Label("Ready")

# ======== Function to send string to USB and update status ========
function send_to_usb(msg::String)
    # Open and configure serial port
    sp = LibSerialPort.open(PORT, BAUD)
    # (optional) configure flow control or flush buffers
    # LibSerialPort.set_flow_control(sp)  # if needed
    # LibSerialPort.sp_flush(sp, SP_BUF_BOTH)  # if desired

    # Write the message
    write(sp, msg * "\n")

    close(sp)

    # Update the GUI label
    set_gtk_property!(status_label, :label, "Sent: $msg")
end


# ======== Create Buttons ========
btnA = Button("FORWARD")
btnB = Button("LEFT")
btnC = Button("RIGHT")
btnD = Button("REVERSE")
btnE = Button("STOP")
btnF = Button("RESET")

# ======== Connect Buttons to USB messages ========
signal_connect(btnA, "clicked") do _; send_to_usb("\$FWD,1600"); end
signal_connect(btnB, "clicked") do _; send_to_usb("\$LFT,1600"); end
signal_connect(btnC, "clicked") do _; send_to_usb("\$RGT,1400"); end
signal_connect(btnD, "clicked") do _; send_to_usb("\$BWD,1400"); end
signal_connect(btnE, "clicked") do _; send_to_usb("\$STP,1500"); end
signal_connect(btnF, "clicked") do _; send_to_usb("\$RST,0000"); end

# ======== Place Buttons in Grid (circular layout) ========
grid[1, 0] = btnA   # Top-center
grid[0, 1] = btnB   # Below-left of A
grid[2, 1] = btnC   # Below-right of A
grid[1, 2] = btnD   # Bottom-center
grid[1, 1] = btnE   # Center

# ======== Add Status Label Below Grid ========
grid[1, 3] = status_label

# ========= Add RESET Button ==========
grid[2,6] = btnF
# ======== Add Grid to Window ========
set_gtk_property!(win, :child, grid)
showall(win)

# ======== Ensure GUI exits cleanly ========
signal_connect(win, "destroy") do _
    Gtk.gtk_quit()
end

# ======== Start GTK event loop ========
Gtk.gtk_main()
