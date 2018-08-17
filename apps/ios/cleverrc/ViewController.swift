//
//  ViewController.swift
//  cleverrc
//
//  Created by Oleg Kalachev on 20.01.2018.
//  Copyright © 2018 Copter Express. All rights reserved.
//

import UIKit
import WebKit
import SwiftSocket
import NotificationBannerSwift
import AudioToolbox.AudioServices

class ViewController: UIViewController, WKScriptMessageHandler {
    @IBOutlet weak var webView: WKWebView!
    let impactGenerator = UIImpactFeedbackGenerator(style: .medium)
    let notificationGenerator = UINotificationFeedbackGenerator()
    let udpSocket = UDPClient(address: "255.255.255.255", port: 35602)

    override func viewDidLoad() {
        super.viewDidLoad()

        // Don't lock screen
        UIApplication.shared.isIdleTimerDisabled = true

        // Setup webview event handlers
        webView.scrollView.bounces = false;
        webView.configuration.userContentController.add(self, name: "control")
        webView.configuration.userContentController.add(self, name: "controlStart")
        webView.configuration.userContentController.add(self, name: "lowBattery")
        webView.configuration.userContentController.add(self, name: "notification")

        // Load the main page
        let url = Bundle.main.url(forResource: "index", withExtension: "html")
        let requestObj = URLRequest(url: url!)
        webView.load(requestObj)

        // Setup UDP broadcasting
        udpSocket.enableBroadcast()

        // Set UDP broadcasting interface
        var wifiInterface = if_nametoindex("en0");
        setsockopt(udpSocket.fd!, IPPROTO_IP, IP_BOUND_IF, &wifiInterface, socklen_t(MemoryLayout<UInt32>.size));
    }

    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
        // Dispose of any resources that can be recreated.
    }

    func userContentController(_ userContentController: WKUserContentController, didReceive message: WKScriptMessage) {
        if (message.name == "control") {
            // Send UDP control message
            let m = message.body as! NSDictionary;
            let d = pack("<hhhh", [m["x"]!, m["y"]!, m["z"]!, m["r"]!])
            _ = udpSocket.send(data: d)
        } else if (message.name == "lowBattery") {
            // Got low battery notification
            print("Low battery notification")
            tapticNotify()
        } else if (message.name == "notification") {
            // Got notification message
            print(message)
            tapticNotify()
        }
    }

    func tapticNotify() {
        if let feedbackSupportLevel = UIDevice.current.value(forKey: "_feedbackSupportLevel") as? Int {
            switch feedbackSupportLevel {
            case 2:
                // 2nd Generation Taptic Engine w/ Haptic Feedback (iPhone 7/7+)
                notificationGenerator.notificationOccurred(.warning)
            case 1:
                // 1st Generation Taptic Engine (iPhone 6S/6S+)
                let peek = SystemSoundID(1519)
                AudioServicesPlaySystemSound(peek)
            case 0:
                // No Taptic Engine
                break
            default: break
            }
        }
    }
}
