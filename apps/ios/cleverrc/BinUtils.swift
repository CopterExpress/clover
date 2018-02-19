//
//  BinUtils.swift
//  BinUtils
//
//  Created by Nicolas Seriot on 12/03/16.
//  Copyright © 2016 Nicolas Seriot. All rights reserved.
//

import Foundation
import CoreFoundation

// MARK: protocol UnpackedType

public protocol Unpackable {}

extension NSString: Unpackable {}
extension Bool: Unpackable {}
extension Int: Unpackable {}
extension Double: Unpackable {}

// MARK: protocol DataConvertible

protocol DataConvertible {}

extension DataConvertible {
    
    init?(data: Data) {
        guard data.count == MemoryLayout<Self>.size else { return nil }
        self = data.withUnsafeBytes { $0.pointee }
    }

    init?(bytes: [UInt8]) {
        let data = Data(bytes:bytes)
        self.init(data:data)
    }
    
    var data: Data {
        var value = self
        return Data(buffer: UnsafeBufferPointer(start: &value, count: 1))
    }
}

extension Bool : DataConvertible { }

extension Int8 : DataConvertible { }
extension Int16 : DataConvertible { }
extension Int32 : DataConvertible { }
extension Int64 : DataConvertible { }

extension UInt8 : DataConvertible { }
extension UInt16 : DataConvertible { }
extension UInt32 : DataConvertible { }
extension UInt64 : DataConvertible { }

extension Float32 : DataConvertible { }
extension Float64 : DataConvertible { }

// MARK: String extension

extension String {
    subscript (from:Int, to:Int) -> String {
        return NSString(string: self).substring(with: NSMakeRange(from, to-from))
    }
}

// MARK: Data extension

extension Data {
    var bytes : [UInt8] {
        return self.withUnsafeBytes {
            [UInt8](UnsafeBufferPointer(start: $0, count: self.count))
        }
    }
}

// MARK: functions

public func hexlify(_ data:Data) -> String {
    
    // similar to hexlify() in Python's binascii module
    // https://docs.python.org/2/library/binascii.html
    
    var s = String()
    var byte: UInt8 = 0
    
    for i in 0 ..< data.count {
        NSData(data: data).getBytes(&byte, range: NSMakeRange(i, 1))
        s = s.appendingFormat("%02x", byte)
    }
    
    return s as String
}

public func unhexlify(_ string:String) -> Data? {
    
    // similar to unhexlify() in Python's binascii module
    // https://docs.python.org/2/library/binascii.html
    
    let s = string.uppercased().replacingOccurrences(of: " ", with: "")
    
    let nonHexCharacterSet = CharacterSet(charactersIn: "0123456789ABCDEF").inverted
    if let range = s.rangeOfCharacter(from: nonHexCharacterSet) {
        print("-- found non hex character at range \(range)")
        return nil
    }
    
    var data = Data(capacity: s.count / 2)
    
    for i in stride(from: 0, to:s.count, by:2) {
        let byteString = s[i, i+2]
        let byte = UInt8(byteString.withCString { strtoul($0, nil, 16) })
        data.append([byte] as [UInt8], count: 1)
    }
    
    return data
}

func readIntegerType<T:DataConvertible>(_ type:T.Type, bytes:[UInt8], loc:inout Int) -> T {
    let size = MemoryLayout<T>.size
    let sub = Array(bytes[loc..<(loc+size)])
    loc += size
    return T(bytes: sub)!
}

func readFloatingPointType<T:DataConvertible>(_ type:T.Type, bytes:[UInt8], loc:inout Int, isBigEndian:Bool) -> T {
    let size = MemoryLayout<T>.size
    let sub = Array(bytes[loc..<(loc+size)])
    loc += size
    let sub_ = isBigEndian ? sub.reversed() : sub
    return T(bytes: sub_)!
}

func isBigEndianFromMandatoryByteOrderFirstCharacter(_ format:String) -> Bool {
    
    guard let firstChar = format.first else { assertionFailure("empty format"); return false }
    
    let s = NSString(string: String(firstChar))
    let c = s.substring(to: 1)
    
    if c == "@" { assertionFailure("native size and alignment is unsupported") }
    
    if c == "=" || c == "<" { return false }
    if c == ">" || c == "!" { return true }
    
    assertionFailure("format '\(format)' first character must be among '=<>!'")
    
    return false
}

// akin to struct.calcsize(fmt)
func numberOfBytesInFormat(_ format:String) -> Int {
    
    var numberOfBytes = 0
    
    var n = 0 // repeat counter
    
    var mutableFormat = format
    
    while !mutableFormat.isEmpty {
        
        let c = mutableFormat.remove(at: mutableFormat.startIndex)
        
        if let i = Int(String(c)) , 0...9 ~= i {
            if n > 0 { n *= 10 }
            n += i
            continue
        }
        
        if c == "s" {
            numberOfBytes += max(n,1)
            n = 0
            continue
        }
        
        for _ in 0..<max(n,1) {
            
            switch(c) {
                
            case "@", "<", "=", ">", "!", " ":
                ()
            case "c", "b", "B", "x", "?":
                numberOfBytes += 1
            case "h", "H":
                numberOfBytes += 2
            case "i", "l", "I", "L", "f":
                numberOfBytes += 4
            case "q", "Q", "d":
                numberOfBytes += 8
            case "P":
                numberOfBytes += MemoryLayout<Int>.size
            default:
                assertionFailure("-- unsupported format \(c)")
            }
        }
        
        n = 0
    }
    
    return numberOfBytes
}

func formatDoesMatchDataLength(_ format:String, data:Data) -> Bool {
    let sizeAccordingToFormat = numberOfBytesInFormat(format)
    let dataLength = data.count
    if sizeAccordingToFormat != dataLength {
        print("format \"\(format)\" expects \(sizeAccordingToFormat) bytes but data is \(dataLength) bytes")
        return false
    }
    
    return true
}

/*
 pack() and unpack() should behave as Python's struct module https://docs.python.org/2/library/struct.html BUT:
 - native size and alignment '@' is not supported
 - as a consequence, the byte order specifier character is mandatory and must be among "=<>!"
 - native byte order '=' assumes a little-endian system (eg. Intel x86)
 - Pascal strings 'p' and native pointers 'P' are not supported
 */

public enum BinUtilsError: Error {
    case formatDoesMatchDataLength(format:String, dataSize:Int)
    case unsupportedFormat(character:Character)
}

public func pack(_ format:String, _ objects:[Any], _ stringEncoding:String.Encoding=String.Encoding.windowsCP1252) -> Data {
    
    var objectsQueue = objects
    
    var mutableFormat = format
    
    var mutableData = Data()
    
    var isBigEndian = false
    
    let firstCharacter = mutableFormat.remove(at: mutableFormat.startIndex)
    
    switch(firstCharacter) {
    case "<", "=":
        isBigEndian = false
    case ">", "!":
        isBigEndian = true
    case "@":
        assertionFailure("native size and alignment '@' is unsupported'")
    default:
        assertionFailure("unsupported format chacracter'")
    }
    
    var n = 0 // repeat counter
    
    while !mutableFormat.isEmpty {
        
        let c = mutableFormat.remove(at: mutableFormat.startIndex)
        
        if let i = Int(String(c)) , 0...9 ~= i {
            if n > 0 { n *= 10 }
            n += i
            continue
        }
        
        var o : Any = 0
        
        if c == "s" {
            o = objectsQueue.remove(at: 0)
            
            guard let stringData = (o as! String).data(using: .utf8) else { assertionFailure(); return Data() }
            var bytes = stringData.bytes
            
            let expectedSize = max(1, n)
            
            // pad ...
            while bytes.count < expectedSize { bytes.append(0x00) }
            
            // ... or trunk
            if bytes.count > expectedSize { bytes = Array(bytes[0..<expectedSize]) }
            
            assert(bytes.count == expectedSize)
            
            if isBigEndian { bytes = bytes.reversed() }
            
            mutableData.append(bytes, count: bytes.count)
            
            n = 0
            continue
        }
        
        for _ in 0..<max(n,1) {
            
            var bytes : [UInt8] = []
            
            if c != "x" {
                o = objectsQueue.removeFirst()
            }
            
            switch(c) {
            case "?":
                bytes = (o as! Bool) ? [0x01] : [0x00]
            case "c":
                let charAsString = (o as! NSString).substring(to: 1)
                guard let data = charAsString.data(using: stringEncoding) else {
                    assertionFailure("cannot decode character \(charAsString) using encoding \(stringEncoding)")
                    return Data()
                }
                bytes = data.bytes
            case "b":
                bytes = Int8(truncatingIfNeeded:o as! Int).data.bytes
            case "h":
                bytes = Int16(truncatingIfNeeded:o as! Int).data.bytes
            case "i", "l":
                bytes = Int32(truncatingIfNeeded:o as! Int).data.bytes
            case "q", "Q":
                bytes = Int64(o as! Int).data.bytes
            case "B":
                bytes = UInt8(truncatingIfNeeded:o as! Int).data.bytes
            case "H":
                bytes = UInt16(truncatingIfNeeded:o as! Int).data.bytes
            case "I", "L":
                bytes = UInt32(truncatingIfNeeded:o as! Int).data.bytes
            case "f":
                bytes = Float32(o as! Double).data.bytes
            case "d":
                bytes = Float64(o as! Double).data.bytes
            case "x":
                bytes = [0x00]
            default:
                assertionFailure("Unsupported packing format: \(c)")
            }
            
            if isBigEndian { bytes = bytes.reversed() }
            let data = Data(bytes)
            mutableData.append(data)
        }
        
        n = 0
    }
    
    return mutableData
}

public func unpack(_ format:String, _ data:Data, _ stringEncoding:String.Encoding=String.Encoding.windowsCP1252) throws -> [Unpackable] {
    
    assert(CFByteOrderGetCurrent() == 1 /* CFByteOrderLittleEndian */, "\(#file) assumes little endian, but host is big endian")
    
    let isBigEndian = isBigEndianFromMandatoryByteOrderFirstCharacter(format)
    
    if formatDoesMatchDataLength(format, data: data) == false {
        throw BinUtilsError.formatDoesMatchDataLength(format:format, dataSize:data.count)
    }
    
    var a : [Unpackable] = []
    
    var loc = 0
    
    let bytes = data.bytes
    
    var n = 0 // repeat counter
    
    var mutableFormat = format
    
    mutableFormat.remove(at: mutableFormat.startIndex) // consume byte-order specifier
    
    while !mutableFormat.isEmpty {
        
        let c = mutableFormat.remove(at: mutableFormat.startIndex)
        
        if let i = Int(String(c)) , 0...9 ~= i {
            if n > 0 { n *= 10 }
            n += i
            continue
        }
        
        if c == "s" {
            let length = max(n,1)
            let sub = Array(bytes[loc..<loc+length])
            
            guard let s = NSString(bytes: sub, length: length, encoding: stringEncoding.rawValue) else {
                assertionFailure("-- not a string: \(sub)")
                return []
            }
            
            a.append(s)
            
            loc += length
            
            n = 0
            
            continue
        }
        
        for _ in 0..<max(n,1) {
            
            var o : Unpackable?
            
            switch(c) {
                
            case "c":
                let optionalString = NSString(bytes: [bytes[loc]], length: 1, encoding: String.Encoding.utf8.rawValue)
                loc += 1
                guard let s = optionalString else { assertionFailure(); return [] }
                o = s
            case "b":
                let r = readIntegerType(Int8.self, bytes:bytes, loc:&loc)
                o = Int(r)
            case "B":
                let r = readIntegerType(UInt8.self, bytes:bytes, loc:&loc)
                o = Int(r)
            case "?":
                let r = readIntegerType(Bool.self, bytes:bytes, loc:&loc)
                o = r ? true : false
            case "h":
                let r = readIntegerType(Int16.self, bytes:bytes, loc:&loc)
                o = Int(isBigEndian ? Int16(bigEndian: r) : r)
            case "H":
                let r = readIntegerType(UInt16.self, bytes:bytes, loc:&loc)
                o = Int(isBigEndian ? UInt16(bigEndian: r) : r)
            case "i":
                fallthrough
            case "l":
                let r = readIntegerType(Int32.self, bytes:bytes, loc:&loc)
                o = Int(isBigEndian ? Int32(bigEndian: r) : r)
            case "I":
                fallthrough
            case "L":
                let r = readIntegerType(UInt32.self, bytes:bytes, loc:&loc)
                o = Int(isBigEndian ? UInt32(bigEndian: r) : r)
            case "q":
                let r = readIntegerType(Int64.self, bytes:bytes, loc:&loc)
                o = Int(isBigEndian ? Int64(bigEndian: r) : r)
            case "Q":
                let r = readIntegerType(UInt64.self, bytes:bytes, loc:&loc)
                o = Int(isBigEndian ? UInt64(bigEndian: r) : r)
            case "f":
                let r = readFloatingPointType(Float32.self, bytes:bytes, loc:&loc, isBigEndian:isBigEndian)
                o = Double(r)
            case "d":
                let r = readFloatingPointType(Float64.self, bytes:bytes, loc:&loc, isBigEndian:isBigEndian)
                o = Double(r)
            case "x":
                loc += 1
            case " ":
                ()
            default:
                throw BinUtilsError.unsupportedFormat(character:c)
            }
            
            if let o = o { a.append(o) }
        }
        
        n = 0
    }
    
    return a
}
