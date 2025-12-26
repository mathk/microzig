//basic DMA support for STM32F1xx

//NOTE: the current bdma-v1 does not include the DMA channel cluster in the DMA struct
//this happens because the DMA controller of this version does not have a fixed number of channels
//that is, it can have any amount between 1 and 7 channels
//for example, DMA2 of STM32F10x high/XL has only 5 channels
//this may have affected the code generation in bdma_v1
const std = @import("std");
const microzig = @import("microzig");
const util = @import("../common/util.zig");
pub const Instances = @import("./enums.zig").DMA_V1_Type;

const hal = microzig.hal;
const Bdma_v1 = microzig.chip.types.peripherals.bdma_v1;
const PriorityLevel = Bdma_v1.PL;
const DIrection = Bdma_v1.DIR;
const Size = Bdma_v1.SIZE;
const Channel_t = Bdma_v1.CH;
const DMA_t = Bdma_v1.DMA;

fn get_regs(comptime instance: Instances) *volatile DMA_t {
    return @field(microzig.chip.peripherals, @tagName(instance));
}

pub const Error = error{
    BufferOverflow,
};

pub const ChannelNumber = enum(u3) {
    Channel1,
    Channel2,
    Channel3,
    Channel4,
    Channel5,
    Channel6,
    Channel7,
};

pub const Config = struct {

    //Channel Configuration
    priority: PriorityLevel = .Low,
    direction: DIrection = .FromPeripheral,
    peripheral_size: Size = .Bits8,
    memory_size: Size = .Bits8,

    //Channel Control Flags
    peripheral_increment: bool = false,
    memory_increment: bool = false,
    circular_mode: bool = false,
    memory_to_memory: bool = false,
    transfer_complete_interrupt: bool = false,
    half_transfer_interrupt: bool = false,
    transfer_error_interrupt: bool = false,

    //Channel Transfer Parameters
    mem_address: ?u32 = null,
    periph_address: u32,
    transfer_count: ?u16 = null,
};
pub const ChannelEvent = packed struct(u4) {
    pending_event: bool,
    transfer_complete: bool,
    half_transfer: bool,
    transfer_error: bool,
};

// Buffer place in region that is accesssible to
extern var dma_buffer: [100]u8;

const DMA1_InteruptName = .{
    "DMA1_Channel1",
    "DMA1_Channel2",
    "DMA1_Channel3",
    "DMA1_Channel4",
    "DMA1_Channel5",
    "DMA1_Channel6",
    "DMA1_Channel7",
};

const DMA2_InteruptName = .{
    "DMA2_Channel1",
    "DMA2_Channel2",
    "DMA2_Channel3",
    "DMA2_Channel4",
    "DMA2_Channel5",
    "DMA2_Channel6",
    "DMA2_Channel7",
};

const DMA_InterputName = .{
    DMA1_InteruptName,
    DMA2_InteruptName,
};

pub fn DMA(comptime dma_ctrl: Instances, comptime ch: ChannelNumber) type {
    const reg_dma = get_regs(dma_ctrl);

    const interupt_name = DMA_InterputName[@intFromEnum(dma_ctrl) - 1][@intFromEnum(ch)];
    const interupt_index = blk: for (microzig.chip.interrupts) |*interrupt| {
        if (std.mem.eql(u8, interupt_name, interrupt.name)) {
            break :blk interrupt.index;
        }
    } else @panic("Interrupt index not found");

    return struct {
        var channel: ?Channel = null;
        var last_event: ?ChannelEvent = null;

        pub fn DMA_Handler() callconv(.c) void {
            last_event = read_events();
            clear_events(last_event.?);
        }

        pub fn clear_events(events: ChannelEvent) void {
            const ch_evt_idx: u5 = 4 * @as(u5, @intCast(@intFromEnum(ch)));
            const bits: u32 = @as(u4, @bitCast(events));
            reg_dma.IFCR.raw |= (bits & 0xF) << ch_evt_idx;
        }

        pub fn read_events() ChannelEvent {
            const ch_evt_idx: u5 = 4 * @as(u5, @intCast(@intFromEnum(ch)));
            return @bitCast(@as(u4, @truncate(reg_dma.ISR.raw >> ch_evt_idx)));
        }

        pub fn get_last_event() ?ChannelEvent {
            return last_event;
        }

        pub fn enable_interrupt() void {
            microzig.interrupt.enable(@as(microzig.cpu.ExternalInterrupt, @enumFromInt(interupt_index)));
        }

        /// NOTE: Channels are 0-Indexed in this API (1..7 [on datasheet] == 0..6)
        pub fn get_channel() *Channel {
            if (channel) |*init_ch| {
                return init_ch;
            }
            hal.rcc.enable_dma(dma_ctrl);

            const channel_base: usize = @intFromPtr(reg_dma) + 0x8 + (20 * @as(usize, @intFromEnum(ch)));

            channel = Channel{
                .reg_channel = @ptrFromInt(channel_base),
            };
            return &channel.?;
        }
    };
}

pub const Channel = struct {
    const Self = @This();
    reg_channel: *volatile Channel_t,

    /// Channel configuration
    ///
    /// NOTE: this function disables the DMA channel, you must use `start()` to start the channel
    pub fn apply(self: *const Self, config: Config) void {
        self.reg_channel.CR.modify(.{
            .EN = 0, //force disable channel before changing config MAR PAR and CNTR
            .DIR = config.direction,
            .CIRC = @intFromBool(config.circular_mode),
            .PINC = @intFromBool(config.peripheral_increment),
            .MINC = @intFromBool(config.memory_increment),
            .PSIZE = config.peripheral_size,
            .MSIZE = config.memory_size,
            .PL = config.priority,
            .MEM2MEM = @intFromBool(config.memory_to_memory),
            .TCIE = @intFromBool(config.transfer_complete_interrupt),
            .HTIE = @intFromBool(config.half_transfer_interrupt),
            .TEIE = @intFromBool(config.transfer_error_interrupt),
        });

        if (config.mem_address) |address| {
            self.reg_channel.MAR = address;
        }
        if (config.transfer_count) |count| {
            self.reg_channel.NDTR.modify_one("NDT", count);
        }
        self.reg_channel.PAR = config.periph_address;
    }

    pub fn start(self: *const Self) void {
        self.reg_channel.CR.modify_one("EN", 1);
    }

    pub fn stop(self: *const Self) void {
        self.reg_channel.CR.modify_one("EN", 0);
    }

    pub fn is_start(self: *const Self) bool {
        return self.reg_channel.CR.read().EN == 1;
    }

    pub fn load_memeory_data(self: *const Self, buffer: []const u8) Error!void {
        if (buffer.len > dma_buffer.len) {
            return Error.BufferOverflow;
        }
        @memcpy(dma_buffer[0..buffer.len], buffer);

        self.reg_channel.NDTR.modify_one("NDT", @as(u16, @intCast(buffer.len)));
        self.reg_channel.MAR = @intFromPtr(&dma_buffer);
    }

    /// Reads the number of remaining transfers.
    /// 0 == DMA has finished all transfers.
    pub inline fn channel_remain_count(self: *const Self) u16 {
        return self.reg_channel.NDTR.read().NDT;
    }
};

// pub const Channel = struct {
//     extern var dma_buffer: [100]u8;
//     reg_dma: *volatile DMA,
//     reg_channel: *volatile Channel_t,
//     ch_num: ChannelNumber,
//     dma_instance: Instances,

//     /// NOTE: Channels are 0-Indexed in this API (1..7 [on datasheet] == 0..6)
//     pub fn init(comptime dma_ctrl: Instances, comptime ch: ChannelNumber) Channel {
//         const reg_dma = get_regs(dma_ctrl);
//         const channel_base: usize = @intFromPtr(reg_dma) + 0x8 + (20 * @as(usize, @intFromEnum(ch)));
//         return Channel{
//             .reg_dma = reg_dma,
//             .reg_channel = @ptrFromInt(channel_base),
//             .ch_num = ch,
//             .dma_instance = dma_ctrl,
//         };
//     }

//     pub fn clear_events(self: *const Channel, events: ChannelEvent) void {
//         const ch_evt_idx: u5 = 4 * @as(u5, @intCast(@intFromEnum(self.ch_num)));
//         const bits: u32 = @as(u4, @bitCast(events));
//         self.reg_dma.IFCR.raw |= (bits & 0xF) << ch_evt_idx;
//     }

//     pub fn read_events(self: *const Channel) ChannelEvent {
//         const ch_evt_idx: u5 = 4 * @as(u5, @intCast(@intFromEnum(self.ch_num)));
//         return @bitCast(@as(u4, @truncate(self.reg_dma.ISR.raw >> ch_evt_idx)));
//     }

//     /// Channel configuration
//     ///
//     /// NOTE: this function disables the DMA channel, you must use `start()` to start the channel
//     pub fn apply(self: *const Channel, config: Config) void {
//         hal.rcc.enable_dma(self.dma_instance);

//         self.reg_channel.CR.modify(.{
//             .EN = 0, //force disable channel before changing config MAR PAR and CNTR
//             .DIR = config.direction,
//             .CIRC = @intFromBool(config.circular_mode),
//             .PINC = @intFromBool(config.peripheral_increment),
//             .MINC = @intFromBool(config.memory_increment),
//             .PSIZE = config.peripheral_size,
//             .MSIZE = config.memory_size,
//             .PL = config.priority,
//             .MEM2MEM = @intFromBool(config.memory_to_memory),
//             .TCIE = @intFromBool(config.transfer_complete_interrupt),
//             .HTIE = @intFromBool(config.half_transfer_interrupt),
//             .TEIE = @intFromBool(config.transfer_error_interrupt),
//         });

//         if (config.mem_address) |address| {
//             self.reg_channel.MAR = address;
//         }
//         if (config.transfer_count) |count| {
//             self.reg_channel.NDTR.modify_one("NDT", count);
//         }
//         self.reg_channel.PAR = config.periph_address;
//     }

//     pub fn start(self: *const Channel) void {
//         self.reg_channel.CR.modify_one("EN", 1);
//     }

//     pub fn stop(self: *const Channel) void {
//         self.reg_channel.CR.modify_one("EN", 0);
//     }

//     pub fn is_start(self: *const Channel) bool {
//         return self.reg_channel.CR.read().EN == 1;
//     }

//     pub fn load_memeory_data(self: *const Channel, buffer: []const u8) Error!void {
//         if (buffer.len > dma_buffer.len) {
//             return Error.BufferOverflow;
//         }
//         @memcpy(dma_buffer[0..buffer.len], buffer);

//         self.reg_channel.NDTR.modify_one("NDT", @as(u16, @intCast(buffer.len)));
//         self.reg_channel.MAR = @intFromPtr(&dma_buffer);
//     }

//     ///changes the memory address.
//     ///
//     /// NOTE: this function temporarily disables the channel
//     pub fn set_memory_address(self: *const Channel, MA: u32) void {
//         const current_en = self.reg_channel.CR.read().EN;

//         // disables the channel before configuring a new value for count
//         self.reg_channel.CR.modify_one("EN", 0);
//         self.reg_channel.MAR = MA;
//         self.reg_channel.CR.modify_one("EN", current_en);
//     }

//     /// Changes the number of transfers.
//     ///
//     /// NOTE: this function temporarily disables the channel
//     pub fn set_count(self: *const Channel, count: u16) void {
//         const current_en = self.reg_channel.CR.read().EN;

//         // disables the channel before configuring a new value for count
//         self.reg_channel.CR.modify_one("EN", 0);
//         self.reg_channel.NDTR.modify_one("NDT", count);
//         self.reg_channel.CR.modify_one("EN", current_en);
//     }

//     /// Reads the number of remaining transfers.
//     /// 0 == DMA has finished all transfers.
//     pub inline fn channel_remain_count(self: *const Channel) u16 {
//         return self.reg_channel.NDTR.read().NDT;
//     }
// };
