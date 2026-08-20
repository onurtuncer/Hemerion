// ------------------------------------------------------------------------------
// Project: Hemerion Copyright (c) 2026, Onur Tuncer, PhD, Istanbul Technical University
//
// SPDX-License-Identifier: GPL-3.0-only License-Filename: LICENSE
// ------------------------------------------------------------------------------
//
// Renode I2C peripheral forwarding transactions to a Hemerion shared-memory
// I2C bus through i2c_shm_tcp_bridge (see DESIGN.md in this directory and the
// wire protocol in sim/i2c_shm/tools/i2c_shm_tcp_bridge.cpp). Loaded at
// runtime -- `include @.../HemerionI2cBridge.cs` before the platform
// description registers it -- so no Renode fork or rebuild is involved:
//
//     bmp390: I2C.HemerionI2cShmBridge @ i2c1 0x76
//         targetAddress: 0x76
//         port: 5767
//
// targetAddress has no default and must repeat the registration address on
// the line above. Renode does not hand an II2CPeripheral its own registration
// address, so the two are genuinely separate values: the controller matches on
// the registration, while this class puts targetAddress on the shm bus. A
// default made them silently disagree -- registering at 0x77 still addressed
// the part at 0x76 -- so the address is required, and Renode reports a missing
// one when the platform is loaded instead of guessing.
//
// Renode's I2C controller models deliver a transfer as Write(bytes) calls, an
// optional Read(count), and FinishTransmission() at STOP. Those are mapped
// onto whole bridge transactions the way the shm bus frames them: written
// bytes accumulate until either a Read arrives (write phase + repeated-START
// read phase, one transaction) or FinishTransmission closes a write-only
// transaction. A Read with nothing accumulated is a pure read -- the device
// model's register pointer persists across transactions, exactly as on the
// real part, so chunked reads still walk the register map correctly.
//
// Failure semantics mirror electrical reality: an address NACK, a bus fault,
// or no bridge at all reads back as 0xFF bytes (an undriven bus floats high)
// and a warning in the Renode log. The emulated firmware then sees a part
// that is absent, which is a state the on-target driver already reports
// cleanly.

using System;
using System.Collections.Generic;
using System.Net.Sockets;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;

namespace Antmicro.Renode.Peripherals.I2C
{
    public class HemerionI2cShmBridge : II2CPeripheral
    {
        public HemerionI2cShmBridge(int targetAddress, int port = 5767, string host = "127.0.0.1")
        {
            this.port = port;
            this.host = host;
            this.targetAddress = (byte)targetAddress;
            pendingWrite = new List<byte>();
        }

        public void Write(byte[] data)
        {
            pendingWrite.AddRange(data);
        }

        public byte[] Read(int count = 1)
        {
            var response = Transact(pendingWrite.ToArray(), count);
            pendingWrite.Clear();
            return response;
        }

        public void FinishTransmission()
        {
            if(pendingWrite.Count > 0)
            {
                Transact(pendingWrite.ToArray(), 0);
                pendingWrite.Clear();
            }
        }

        public void Reset()
        {
            pendingWrite.Clear();
            // Drop the connection too: a machine reset is the right moment to
            // resynchronize with a bridge that may have been restarted -- and
            // to re-arm the connect warning, so the next failure is reported
            // rather than assumed already seen.
            connectFailureReported = false;
            Disconnect();
        }

        private byte[] Transact(byte[] writeBytes, int readCount)
        {
            var floatingBus = FloatingBytes(readCount);
            if(!EnsureConnected())
            {
                return floatingBus;
            }

            try
            {
                var request = new byte[5 + writeBytes.Length];
                request[0] = targetAddress;
                request[1] = (byte)(writeBytes.Length & 0xFF);
                request[2] = (byte)((writeBytes.Length >> 8) & 0xFF);
                request[3] = (byte)(readCount & 0xFF);
                request[4] = (byte)((readCount >> 8) & 0xFF);
                Array.Copy(writeBytes, 0, request, 5, writeBytes.Length);

                var networkStream = client.GetStream();
                networkStream.Write(request, 0, request.Length);

                var result = new byte[1];
                if(!ReceiveExact(networkStream, result, 1))
                {
                    this.Log(LogLevel.Warning, "Bridge connection lost mid-transaction");
                    Disconnect();
                    return floatingBus;
                }
                if(result[0] != 0)
                {
                    // Address/data NACK or bus fault: named in the log, floating on the bus.
                    this.Log(LogLevel.Warning, "Bridge transaction not acknowledged (result {0})", result[0]);
                    return floatingBus;
                }
                if(readCount == 0)
                {
                    return new byte[0];
                }
                var payload = new byte[readCount];
                if(!ReceiveExact(networkStream, payload, readCount))
                {
                    this.Log(LogLevel.Warning, "Bridge connection lost reading the response payload");
                    Disconnect();
                    return floatingBus;
                }
                return payload;
            }
            catch(Exception e)
            {
                this.Log(LogLevel.Warning, "Bridge transaction failed: {0}", e.Message);
                Disconnect();
                return floatingBus;
            }
        }

        private bool EnsureConnected()
        {
            if(client != null && client.Connected)
            {
                return true;
            }
            Disconnect();
            try
            {
                client = new TcpClient();
                client.NoDelay = true;
                // Without these, a bridge that stops answering parks the
                // emulated CPU thread in stream.Read forever -- and nothing
                // rescues it, because Renode's virtual clock is stopped while
                // that thread blocks, so the harness's TerminalTester budget
                // (counted in virtual seconds) never elapses. A timed-out read
                // throws, which Transact already handles: log, drop the
                // connection, float the bus.
                client.ReceiveTimeout = ResponseTimeoutMs;
                client.SendTimeout = ResponseTimeoutMs;
                client.Connect(host, port);
                // Cleared on every success, not just set once. Latched, this
                // suppressed the warning exactly when it mattered: the harness
                // *expects* early transactions to race the bridge's attach and
                // read as absent, which spent the single report, so a bridge
                // dying later in the run left nothing in the log but silent
                // 0xFF reads.
                connectFailureReported = false;
                // The address is in the line too: it is the one value here
                // that the platform description states twice, so an operator
                // comparing the log against the .repl can see a disagreement.
                this.Log(LogLevel.Info, "Connected to the I2C bridge at {0}:{1}, addressing the part at 0x{2:X2}",
                         host, port, targetAddress);
                return true;
            }
            catch(Exception e)
            {
                if(!connectFailureReported)
                {
                    // Once, not per transaction: a probing driver retries at
                    // hundreds of hertz and the log should stay readable.
                    this.Log(LogLevel.Warning, "Cannot reach the I2C bridge at {0}:{1} ({2}); the part reads as absent",
                             host, port, e.Message);
                    connectFailureReported = true;
                }
                Disconnect();
                return false;
            }
        }

        private void Disconnect()
        {
            if(client != null)
            {
                try
                {
                    client.Close();
                }
                catch(Exception)
                {
                    // Closing a broken socket is best-effort by definition.
                }
                client = null;
            }
        }

        private static bool ReceiveExact(NetworkStream stream, byte[] buffer, int length)
        {
            var received = 0;
            while(received < length)
            {
                var chunk = stream.Read(buffer, received, length - received);
                if(chunk <= 0)
                {
                    return false;
                }
                received += chunk;
            }
            return true;
        }

        private static byte[] FloatingBytes(int count)
        {
            var bytes = new byte[count];
            for(var i = 0; i < count; ++i)
            {
                bytes[i] = 0xFF;
            }
            return bytes;
        }

        // i2c_shm_tcp_bridge bounds its own transactions at kTransactionTimeout
        // (1000 ms) even when the simulated part is hung, so a few times that
        // only trips on a bridge that has stopped answering altogether.
        private const int ResponseTimeoutMs = 4000;

        private readonly int port;
        private readonly string host;
        private readonly byte targetAddress;
        private readonly List<byte> pendingWrite;
        private TcpClient client;
        private bool connectFailureReported;
    }
}
